#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "angles/angles.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>

class PathPlanner3DNode : public rclcpp::Node
{
public:
    PathPlanner3DNode() : Node("path_planner_3d_node"), last_best_idx_(-1), smoothed_angle_(0.0)
    {
        // Parameters
        this->declare_parameter<bool>("use_voxel_filter", false); // Default to false
        this->get_parameter("use_voxel_filter", use_voxel_filter_);
        this->declare_parameter<double>("voxel_leaf_size", 0.1);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->declare_parameter<double>("corridor_width", 0.3); // 30cm width for each 3D corridor
        this->get_parameter("corridor_width", corridor_width_);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/downsampled_cloud", 10, std::bind(&PathPlanner3DNode::pointCloudCallback, this, std::placeholders::_1));

        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector_3d", 10);
        preprocessed_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/preprocessed_cloud_3d", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/candidate_rays_3d", 10);

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "3D Path Planner (3D Corridor Scan) Node has been started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Phase 1: Preprocessing (Voxel Filter is optional)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_prepared(new pcl::PointCloud<pcl::PointXYZ>);
        if (use_voxel_filter_) {
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            sor.filter(*cloud_prepared);
        } else {
            *cloud_prepared = *cloud;
        }
        
        // Publish the (potentially downsampled) cloud for debugging
        sensor_msgs::msg::PointCloud2 preprocessed_msg;
        pcl::toROSMsg(*cloud_prepared, preprocessed_msg);
        preprocessed_msg.header = msg->header;
        preprocessed_publisher_->publish(preprocessed_msg);


        // Phase 2: 3D Corridor Scan
        const int num_rays = 15;
        const double max_dist = 5.0;
        std::vector<double> depths(num_rays, max_dist);

        for (int i = 0; i < num_rays; ++i) {
            double angle = (i * (M_PI / (num_rays - 1))) - (M_PI / 2.0);
            Eigen::Vector2f ray_dir(cos(angle), sin(angle));
            double min_dist_for_ray = max_dist;

            for (const auto& point : *cloud_prepared) {
                // Basic front view filter
                if (point.x < 0) continue;

                Eigen::Vector2f point_vec_2d(point.x, point.y);
                
                // Check if the point is within the 3D corridor
                if (point_vec_2d.dot(ray_dir) > 0) {
                    // Calculate the perpendicular distance from the point to the ray's line
                    double dist_to_ray = std::abs(point.x * ray_dir.y() - point.y * ray_dir.x());
                    
                    if (dist_to_ray < (corridor_width_ / 2.0)) {
                        // The point is inside the corridor's width
                        // The distance to this obstacle is its distance from the sensor
                        double point_dist = point_vec_2d.norm();
                        min_dist_for_ray = std::min(min_dist_for_ray, point_dist);
                    }
                }
            }
            depths[i] = min_dist_for_ray;
        }

        // Phase 3: Scoring, Smoothing, and Publishing (Identical to 2D version)
        std::vector<double> scores(num_rays, 0.0);
        double w1 = 0.7;
        double w2 = 0.3;
        for (int i = 0; i < num_rays; ++i) {
            double prev_depth = (i > 0) ? depths[i - 1] : depths[i];
            double next_depth = (i < num_rays - 1) ? depths[i + 1] : depths[i];
            scores[i] = w1 * depths[i] + w2 * (prev_depth + next_depth);
        }

        int best_ray_idx = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));

        // --- Bug Fix: Force forward direction if no obstacles are detected ---
        bool all_max_depth = true;
        for(const auto& depth : depths) {
            if (depth < max_dist) {
                all_max_depth = false;
                break;
            }
        }
        if (all_max_depth) {
            best_ray_idx = num_rays / 2; // Choose the center ray
        }
        // --- End of Bug Fix ---

        double ideal_angle = (best_ray_idx * (M_PI / (num_rays - 1))) - (M_PI / 2.0);

        const double max_angular_velocity = M_PI / 2.0;
        double angle_diff = angles::shortest_angular_distance(smoothed_angle_, ideal_angle);
        double max_angle_change = max_angular_velocity * dt;
        double angle_change = std::clamp(angle_diff, -max_angle_change, max_angle_change);
        
        smoothed_angle_ += angle_change;
        smoothed_angle_ = angles::normalize_angle(smoothed_angle_);

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
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "candidate_rays_3d";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.points.resize(2);
            marker.points[0].x = 0; marker.points[0].y = 0; marker.points[0].z = 0;
            double angle = (i * (M_PI / (num_rays - 1))) - (M_PI / 2.0);
            marker.points[1].x = depths[i] * cos(angle);
            marker.points[1].y = depths[i] * sin(angle);
            marker.points[1].z = 0;
            marker.scale.x = 0.02;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 0.5;
            marker.color.r = (5.0 - depths[i]) / 5.0;
            marker.color.g = depths[i] / 5.0;
            marker.color.b = 0.0;
            marker_array.markers.push_back(marker);
        }

        visualization_msgs::msg::Marker ideal_marker;
        ideal_marker.header = header;
        ideal_marker.ns = "path_vectors_3d";
        ideal_marker.id = num_rays;
        ideal_marker.type = visualization_msgs::msg::Marker::ARROW;
        ideal_marker.action = visualization_msgs::msg::Marker::ADD;
        ideal_marker.points.resize(2);
        ideal_marker.points[0].x = 0; ideal_marker.points[0].y = 0; ideal_marker.points[0].z = 0;
        double ideal_angle = (best_ray_idx * (M_PI / (num_rays - 1))) - (M_PI / 2.0);
        double ideal_depth = depths[best_ray_idx];
        ideal_marker.points[1].x = ideal_depth * cos(ideal_angle);
        ideal_marker.points[1].y = ideal_depth * sin(ideal_angle);
        ideal_marker.points[1].z = 0;
        ideal_marker.scale.x = 0.05;
        ideal_marker.scale.y = 0.1;
        ideal_marker.scale.z = 0.1;
        ideal_marker.color.a = 1.0;
        ideal_marker.color.r = 1.0;
        ideal_marker.color.g = 1.0;
        ideal_marker.color.b = 0.0;
        marker_array.markers.push_back(ideal_marker);

        double smoothed_depth = 5.0; // Default length
        visualization_msgs::msg::Marker smoothed_marker = ideal_marker;
        smoothed_marker.id = num_rays + 1;
        smoothed_marker.points[1].x = smoothed_depth * cos(smoothed_angle);
        smoothed_marker.points[1].y = smoothed_depth * sin(smoothed_angle);
        smoothed_marker.color.r = 0.0;
        smoothed_marker.color.g = 0.5;
        smoothed_marker.color.b = 1.0;
        marker_array.markers.push_back(smoothed_marker);

        marker_publisher_->publish(marker_array);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr preprocessed_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    int last_best_idx_;
    double smoothed_angle_;
    rclcpp::Time last_time_;
    bool use_voxel_filter_;
    double voxel_leaf_size_;
    double corridor_width_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner3DNode>());
    rclcpp::shutdown();
    return 0;
}