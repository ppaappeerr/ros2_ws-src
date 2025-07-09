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
#include <pcl/common/distances.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class GroundPlaneFitterNode : public rclcpp::Node
{
public:
    GroundPlaneFitterNode() : Node("ground_plane_fitter_lock_plane_node"), plane_locked_(false), ransac_count_(0)
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
        this->declare_parameter<int>("ransac_lock_count", 50); // Number of RANSAC iterations before locking

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("refined_ground_topic", refined_ground_topic_);
        this->get_parameter("obstacle_topic", obstacle_topic_);
        this->get_parameter("marker_topic", marker_topic_);
        this->get_parameter("ground_height_threshold_min", ground_height_threshold_min_);
        this->get_parameter("ground_height_threshold_max", ground_height_threshold_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
        this->get_parameter("ransac_lock_count", ransac_lock_count_);

        RCLCPP_INFO(this->get_logger(), "Ground Plane Fitter (Lock Plane Version) Node started.");

        // Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&GroundPlaneFitterNode::cloud_callback, this, std::placeholders::_1));

        // Publishers
        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(refined_ground_topic_, 10);
        obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);

        accumulated_candidates_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        locked_plane_coefficients_ = pcl::make_shared<pcl::ModelCoefficients>();
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty()) return;

        // --- Obstacle Extraction (from original cloud) ---
        // This part remains the same
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

        if (!plane_locked_)
        {
            // --- RANSAC Phase ---
            *accumulated_candidates_ += *ground_candidates;
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(accumulated_candidates_);
            voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_grid.filter(*accumulated_candidates_);

            if (accumulated_candidates_->points.size() < 10) return;

            pcl::ModelCoefficients::Ptr current_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_distance_threshold_);
            seg.setInputCloud(accumulated_candidates_);
            seg.segment(*inliers, *current_coefficients);

            if (inliers->indices.size() == 0) return;

            *locked_plane_coefficients_ = *current_coefficients; // Update potential locked plane
            ransac_count_++;

            if (ransac_count_ > ransac_lock_count_)
            {
                plane_locked_ = true;
                RCLCPP_INFO(this->get_logger(), "Ground plane locked with %d iterations.", ransac_count_);
                accumulated_candidates_->clear(); // No longer needed
            }

            // Publish inliers from RANSAC phase
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
        }
        else
        {
            // --- Plane Locked Phase ---
            pcl::PointCloud<pcl::PointXYZ>::Ptr refined_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& point : ground_candidates->points)
            {
                // Calculate distance from point to the locked plane
                double distance = std::abs(
                    locked_plane_coefficients_->values[0] * point.x +
                    locked_plane_coefficients_->values[1] * point.y +
                    locked_plane_coefficients_->values[2] * point.z +
                    locked_plane_coefficients_->values[3]
                ); // Note: Normal is assumed to be unit vector from RANSAC

                if (distance < ransac_distance_threshold_)
                {
                    refined_ground_cloud->points.push_back(point);
                }
            }
            
            if (!refined_ground_cloud->points.empty()) {
                refined_ground_cloud->width = refined_ground_cloud->points.size();
                refined_ground_cloud->height = 1;
                refined_ground_cloud->is_dense = true;

                sensor_msgs::msg::PointCloud2 ground_msg;
                pcl::toROSMsg(*refined_ground_cloud, ground_msg);
                ground_msg.header = msg->header;
                ground_publisher_->publish(ground_msg);
            }
        }

        // --- Publish Plane Marker using the latest (or locked) coefficients ---
        if (!locked_plane_coefficients_->values.empty()) {
            publish_plane_marker(msg->header, *locked_plane_coefficients_);
        }
    }

    void publish_plane_marker(const std_msgs::msg::Header& header, const pcl::ModelCoefficients& coeffs)
    {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "ground_plane";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        tf2::Vector3 normal(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
        if (normal.length2() < 1e-6) return;
        normal.normalize();

        tf2::Vector3 up(0, 0, 1);
        tf2::Quaternion q;
        tf2::Vector3 axis = normal.cross(up);
        if (axis.length2() < 1e-6) {
            q.setRPY(0, (normal.z() > 0 ? 0 : M_PI), 0);
        } else {
            q.setRotation(axis.normalized(), normal.angle(up));
        }
        
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        if (std::abs(coeffs.values[2]) < 1e-6) return;
        marker.pose.position.z = -coeffs.values[3] / coeffs.values[2];

        marker.scale.x = 10.0;
        marker.scale.y = 10.0;
        marker.scale.z = 0.01;

        marker.color.a = 0.5;
        marker.color.r = plane_locked_ ? 1.0 : 0.0; // Red when locked
        marker.color.g = plane_locked_ ? 0.0 : 1.0; // Green when not locked
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_candidates_;
    pcl::ModelCoefficients::Ptr locked_plane_coefficients_;
    bool plane_locked_;
    int ransac_count_;
    int ransac_lock_count_;

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
