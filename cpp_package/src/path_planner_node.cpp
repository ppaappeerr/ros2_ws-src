#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "angles/angles.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

class PathPlannerNode : public rclcpp::Node
{
public:
    PathPlannerNode() : Node("path_planner_node"), last_best_idx_(-1), smoothed_angle_(M_PI) // initialize looking toward -X
    {
        this->declare_parameter<bool>("front_view_only", true);
        this->get_parameter("front_view_only", front_view_only_);
        this->declare_parameter<bool>("use_voxel_filter", true);
        this->get_parameter("use_voxel_filter", use_voxel_filter_);
        this->declare_parameter<double>("voxel_leaf_size", 0.1);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

        // Parameters for Dead-zone (occlusion) and ROI filtering
        this->declare_parameter<bool>("dead_zone_enabled", true);
        this->declare_parameter<std::string>("mount_side", "right");
        this->declare_parameter<double>("dead_zone_forward_x", 0.25); // How far forward the dead-zone extends
        this->declare_parameter<double>("dead_zone_lateral_y", 0.50); // How far to the side the dead-zone extends
        this->declare_parameter<double>("roi_max_radius", 3.0); // Max distance of points to consider

        this->get_parameter("dead_zone_enabled", dead_zone_enabled_);
        this->get_parameter("mount_side", mount_side_);
        this->get_parameter("dead_zone_forward_x", dead_zone_forward_x_);
        this->get_parameter("dead_zone_lateral_y", dead_zone_lateral_y_);
        this->get_parameter("roi_max_radius", roi_max_radius_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sweep_cloud_cpp", 10, std::bind(&PathPlannerNode::pointCloudCallback, this, std::placeholders::_1));

        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_debug", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Path Planner Node started (front inverted to -X).");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Phase 1: Preprocessing
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // --- New Dead-zone and ROI Filtering ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_roi_filtered->reserve(cloud->size());

        for (const auto& point : *cloud) {
            // 1. ROI radius filtering
            if (std::sqrt(point.x * point.x + point.y * point.y) > roi_max_radius_) {
                continue;
            }

            // 2. Dead-zone filtering
            if (dead_zone_enabled_) {
                bool in_dead_zone = false;
                // Front is -X. Dead zone is in front of the sensor (negative x) 
                // and to one side (positive or negative y).
                if (point.x < 0 && point.x > -dead_zone_forward_x_) {
                    if (mount_side_ == "right" && point.y > 0 && point.y < dead_zone_lateral_y_) {
                        // Right shoulder mount, dead zone is on the left (positive y)
                        in_dead_zone = true;
                    } else if (mount_side_ == "left" && point.y < 0 && point.y > -dead_zone_lateral_y_) {
                        // Left shoulder mount, dead zone is on the right (negative y)
                        in_dead_zone = true;
                    }
                }
                if (in_dead_zone) {
                    continue;
                }
            }
            cloud_roi_filtered->points.push_back(point);
        }
        cloud_roi_filtered->width = cloud_roi_filtered->points.size();
        cloud_roi_filtered->height = 1;
        cloud_roi_filtered->is_dense = true;
        // --- End of New Filtering ---


        // ROI Filtering & Downsampling...
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_roi_filtered); // Use the new filtered cloud

        if (front_view_only_) {
            // NEW: front is -X (negative). Keep points where x in [-5,0]
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-roi_max_radius_, 0.0); // Use roi_max_radius_
            pass.filter(*cloud_filtered);
        } else {
            *cloud_filtered = *cloud_roi_filtered;
        }
        
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.0, 2.0);
        pass.filter(*cloud_filtered);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        if (use_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_filtered);
            sor.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            sor.filter(*cloud_downsampled);
        } else {
            *cloud_downsampled = *cloud_filtered;
        }

        for (auto& point : *cloud_downsampled) {
            point.z = 0;
        }
        // sensor_msgs::msg::PointCloud2 preprocessed_msg;
        // pcl::toROSMsg(*cloud_downsampled, preprocessed_msg);
        // preprocessed_msg.header = msg->header;
        // preprocessed_publisher_->publish(preprocessed_msg);

        // Phase 2: Safety Vector Calculation
        const int num_rays = 15;
        const double max_dist = 5.0;
        std::vector<double> depths(num_rays, max_dist);

        for (int i = 0; i < num_rays; ++i) {
            double base_angle = (i * (M_PI / (num_rays - 1))) - (M_PI / 2.0); // [-pi/2, pi/2] around +X
            double angle = base_angle + M_PI; // shift 180deg so 0 points to -X
            Eigen::Vector2f ray_dir(cos(angle), sin(angle));
            double min_dist_for_ray = max_dist;
            for (const auto& point : *cloud_downsampled) {
                // Skip points BEHIND new forward (-X): i.e., points with x > 0
                if (point.x > 0) continue;
                Eigen::Vector2f point_vec(point.x, point.y);
                if (point_vec.norm() == 0) continue;
                if (point_vec.dot(ray_dir) > 0) {
                    double dist_to_ray = std::abs(point_vec.x() * ray_dir.y() - point_vec.y() * ray_dir.x());
                    if (dist_to_ray < 0.15) {
                        min_dist_for_ray = std::min(min_dist_for_ray, static_cast<double>(point_vec.norm()));
                    }
                }
            }
            depths[i] = min_dist_for_ray;
        }

        std::vector<double> scores(num_rays, 0.0);
        double w1 = 0.7, w2 = 0.3;
        for (int i = 0; i < num_rays; ++i) {
            double prev_depth = (i > 0) ? depths[i - 1] : depths[i];
            double next_depth = (i < num_rays - 1) ? depths[i + 1] : depths[i];
            scores[i] = w1 * depths[i] + w2 * (prev_depth + next_depth);
            // Mild central bias to avoid hugging FOV edges (i=0 & i=end)
            double center_idx = (num_rays - 1) / 2.0;
            double norm_pos = std::abs(i - center_idx) / center_idx;
            scores[i] *= (1.0 - 0.15 * norm_pos * norm_pos);
        }

        // Optimal Raw Vector Selection
        int best_ray_idx = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
        double ideal_angle = ((best_ray_idx * (M_PI / (num_rays - 1))) - (M_PI / 2.0)) + M_PI; // shifted frame

        // Angular Velocity Smoothing
        const double max_angular_velocity = M_PI / 2.0; // 90 deg/s
        double angle_diff = angles::shortest_angular_distance(smoothed_angle_, ideal_angle);
        double max_angle_change = max_angular_velocity * dt;
        double angle_change = std::clamp(angle_diff, -max_angle_change, max_angle_change);
        smoothed_angle_ = angles::normalize_angle(smoothed_angle_ + angle_change);

        // Phase 3: Publish Results
        geometry_msgs::msg::Vector3Stamped safe_vector_msg;
        safe_vector_msg.header = msg->header;
        safe_vector_msg.vector.x = cos(smoothed_angle_);
        safe_vector_msg.vector.y = sin(smoothed_angle_);
        safe_vector_msg.vector.z = 0;
        vector_publisher_->publish(safe_vector_msg);

        publishMarkers(depths, msg->header, best_ray_idx, smoothed_angle_);
    }

    void publishMarkers(const std::vector<double>& depths, const std_msgs::msg::Header& header, int best_ray_idx, double smoothed_angle)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        const int num_rays = depths.size();
        for (int i = 0; i < num_rays; ++i) {
            // ... (Candidate rays marker code is the same, omitted for brevity)
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "candidate_rays";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.points.resize(2);
            marker.points[0].x = 0; marker.points[0].y = 0; marker.points[0].z = 0;
            double angle = ((i * (M_PI / (num_rays - 1))) - (M_PI / 2.0)) + M_PI;
            marker.points[1].x = depths[i] * cos(angle);
            marker.points[1].y = depths[i] * sin(angle);
            marker.points[1].z = 0;
            marker.scale.x = 0.02;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 0.1; // Make candidate rays very transparent
            marker.color.r = (5.0 - depths[i]) / 5.0;
            marker.color.g = depths[i] / 5.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }

        // Yellow marker for the ideal (raw) direction
        visualization_msgs::msg::Marker ideal_marker;
        ideal_marker.header = header;
        ideal_marker.ns = "path_vectors";
        ideal_marker.id = num_rays;
        ideal_marker.type = visualization_msgs::msg::Marker::ARROW;
        ideal_marker.action = visualization_msgs::msg::Marker::ADD;
        ideal_marker.points.resize(2);
        ideal_marker.points[0].x = 0; ideal_marker.points[0].y = 0; ideal_marker.points[0].z = 0;
        double ideal_angle = ((best_ray_idx * (M_PI / (num_rays - 1))) - (M_PI / 2.0)) + M_PI;
        double ideal_depth = depths[best_ray_idx];
        ideal_marker.points[1].x = ideal_depth * cos(ideal_angle);
        ideal_marker.points[1].y = ideal_depth * sin(ideal_angle);
        ideal_marker.points[1].z = 0;
        ideal_marker.scale.x = 0.05; // Reduced thickness
        ideal_marker.scale.y = 0.1;  // Reduced thickness
        ideal_marker.scale.z = 0.1;
        ideal_marker.color.a = 0.1; // Make ideal vector very transparent
        ideal_marker.color.r = 1.0;
        ideal_marker.color.g = 1.0;
        ideal_marker.color.b = 0.0; // Yellow
        marker_array.markers.push_back(ideal_marker);

        // Blue marker for the smoothed final direction
        // Interpolate depth for the smoothed angle
        double smoothed_angle_normalized = smoothed_angle + (M_PI / 2.0); // Range [0, PI]
        double smoothed_ray_idx_float = smoothed_angle_normalized / (M_PI / (num_rays - 1));
        int r_idx1 = std::floor(smoothed_ray_idx_float);
        int r_idx2 = std::ceil(smoothed_ray_idx_float);
        r_idx1 = std::max(0, std::min(num_rays - 1, r_idx1));
        r_idx2 = std::max(0, std::min(num_rays - 1, r_idx2));
        
        double smoothed_depth;
        if (r_idx1 == r_idx2) {
            smoothed_depth = depths[r_idx1];
        } else {
            double t = smoothed_ray_idx_float - r_idx1;
            smoothed_depth = (1.0 - t) * depths[r_idx1] + t * depths[r_idx2];
        }

        visualization_msgs::msg::Marker smoothed_marker = ideal_marker;
        smoothed_marker.id = num_rays + 1;
        smoothed_marker.points[1].x = smoothed_depth * cos(smoothed_angle);
        smoothed_marker.points[1].y = smoothed_depth * sin(smoothed_angle);
        smoothed_marker.color.a = 1.0; // Keep final vector fully opaque
        smoothed_marker.color.r = 0.0;
        smoothed_marker.color.g = 0.5;
        smoothed_marker.color.b = 1.0; // Blue
        marker_array.markers.push_back(smoothed_marker);

        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    int last_best_idx_;
    double smoothed_angle_;
    rclcpp::Time last_time_;
    bool front_view_only_;
    bool use_voxel_filter_;
    double voxel_leaf_size_;

    // Member variables for Dead-zone and ROI
    bool dead_zone_enabled_;
    std::string mount_side_;
    double dead_zone_forward_x_;
    double dead_zone_lateral_y_;
    double roi_max_radius_;
};

// main function is the same

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlannerNode>());
    rclcpp::shutdown();
    return 0;
}