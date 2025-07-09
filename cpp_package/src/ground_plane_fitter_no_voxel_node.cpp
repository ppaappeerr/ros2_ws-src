#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GroundPlaneFitterNode : public rclcpp::Node
{
public:
    GroundPlaneFitterNode() : Node("ground_plane_fitter_no_voxel_node"), first_plane_(true)
    {
        // Parameters
        this->declare_parameter<std::string>("input_topic", "/sweep_cloud_cpp");
        this->declare_parameter<std::string>("refined_ground_topic", "/refined_ground_points");
        this->declare_parameter<std::string>("obstacle_topic", "/obstacle_points");
        this->declare_parameter<std::string>("marker_topic", "/ground_plane_marker");
        this->declare_parameter<double>("ground_height_threshold_min", -1.7);
        this->declare_parameter<double>("ground_height_threshold_max", -1.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.05);
        this->declare_parameter<double>("ransac_distance_threshold", 0.05);
        this->declare_parameter<double>("smoothing_alpha", 0.1); // Smoothing factor

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("refined_ground_topic", refined_ground_topic_);
        this->get_parameter("obstacle_topic", obstacle_topic_);
        this->get_parameter("marker_topic", marker_topic_);
        this->get_parameter("ground_height_threshold_min", ground_height_threshold_min_);
        this->get_parameter("ground_height_threshold_max", ground_height_threshold_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
        this->get_parameter("smoothing_alpha", smoothing_alpha_);

        RCLCPP_INFO(this->get_logger(), "Ground Plane Fitter Node started.");

        // Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&GroundPlaneFitterNode::cloud_callback, this, std::placeholders::_1));

        // Publishers
        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(refined_ground_topic_, 10);
        obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

        accumulated_candidates_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        previous_coefficients_ = pcl::make_shared<pcl::ModelCoefficients>();
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty()) return;

        // --- Obstacle Extraction (from original cloud) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_obs;
        pass_obs.setInputCloud(input_cloud);
        pass_obs.setFilterFieldName("z");
        pass_obs.setFilterLimits(ground_height_threshold_min_, ground_height_threshold_max_);
        pass_obs.setNegative(true);
        pass_obs.filter(*obstacle_cloud);
        
        if (!obstacle_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 obstacle_msg;
            pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
            obstacle_msg.header = msg->header;
            obstacle_publisher_->publish(obstacle_msg);
        }

        // --- Ground Candidate Extraction ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_ground;
        pass_ground.setInputCloud(input_cloud);
        pass_ground.setFilterFieldName("z");
        pass_ground.setFilterLimits(ground_height_threshold_min_, ground_height_threshold_max_);
        pass_ground.filter(*ground_candidates);

        if (ground_candidates->empty()) return;

        // --- Accumulate & Downsample Ground Candidates ---
        *accumulated_candidates_ += *ground_candidates;
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(accumulated_candidates_);
        voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel_grid.filter(*accumulated_candidates_);

        if (accumulated_candidates_->points.size() < 10) return; // Need enough points for RANSAC

        // --- RANSAC Plane Fitting ---
        pcl::ModelCoefficients::Ptr current_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ransac_distance_threshold_);
        seg.setInputCloud(accumulated_candidates_);
        seg.segment(*inliers, *current_coefficients);

        if (inliers->indices.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given dataset.");
            return;
        }

        // --- Temporal Smoothing of Plane Coefficients ---
        if (first_plane_)
        {
            *previous_coefficients_ = *current_coefficients;
            first_plane_ = false;
        }
        else
        {
            for (size_t i = 0; i < current_coefficients->values.size(); ++i)
            {
                previous_coefficients_->values[i] = 
                    smoothing_alpha_ * current_coefficients->values[i] + 
                    (1.0 - smoothing_alpha_) * previous_coefficients_->values[i];
            }
        }

        // --- Publish Refined Ground Points (Inliers) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr refined_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(accumulated_candidates_);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*refined_ground_cloud);

        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*refined_ground_cloud, ground_msg);
        ground_msg.header = msg->header;
        ground_publisher_->publish(ground_msg);

        // --- Publish Plane Marker using smoothed coefficients ---
        publish_plane_marker(msg->header, *previous_coefficients_);
    }

    void publish_plane_marker(const std_msgs::msg::Header& header, const pcl::ModelCoefficients& coeffs)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "ground_plane";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Plane equation: ax + by + cz + d = 0
        // Normal vector is (a, b, c)
        tf2::Vector3 normal(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
        // Ensure normal is not zero
        if (normal.length2() < 1e-6) {
            RCLCPP_WARN(this->get_logger(), "Plane normal vector is close to zero, cannot publish marker.");
            return;
        }
        normal.normalize();

        tf2::Vector3 up(0, 0, 1);
        tf2::Quaternion q;
        
        tf2::Vector3 axis = normal.cross(up);
        // Check if normal is parallel to up vector
        if (axis.length2() < 1e-6) {
            // If normal is pointing up, no rotation needed. If pointing down, 180 deg rotation.
            q.setRPY(0, (normal.z() > 0 ? 0 : M_PI), 0);
        } else {
            q.setRotation(axis.normalized(), normal.angle(up));
        }
        
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Position the marker at the center of the point cloud on the plane
        // For simplicity, we place it at a certain distance along the normal
        // A better approach would be to find the centroid of the inliers
        marker.pose.position.x = 0; // Simplified position
        marker.pose.position.y = 0;
        // Ensure c-coefficient is not zero before dividing
        if (std::abs(coeffs.values[2]) < 1e-6) {
             RCLCPP_WARN(this->get_logger(), "C coefficient is close to zero, cannot determine marker z position.");
             return;
        }
        marker.pose.position.z = -coeffs.values[3] / coeffs.values[2]; // z = -d/c when x=y=0

        marker.scale.x = 10.0;
        marker.scale.y = 10.0;
        marker.scale.z = 0.01;

        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_candidates_;
    pcl::ModelCoefficients::Ptr previous_coefficients_;
    bool first_plane_;
    double smoothing_alpha_;

    std::string input_topic_, refined_ground_topic_, obstacle_topic_, marker_topic_;
    double ground_height_threshold_min_, ground_height_threshold_max_, voxel_leaf_size_, ransac_distance_threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GroundPlaneFitterNode>());
    rclcpp::shutdown();
    return 0;
}
