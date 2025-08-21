#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h" // Required for front view filter

class VoxelGridFilterNode : public rclcpp::Node
{
public:
    VoxelGridFilterNode() : Node("voxel_grid_filter_node")
    {
        // Parameters
        this->declare_parameter<double>("leaf_size", 0.1);
        this->get_parameter("leaf_size", leaf_size_);
        this->declare_parameter<std::string>("input_topic", "/sweep_cloud_cpp");
        this->get_parameter("input_topic", input_topic_);
        this->declare_parameter<std::string>("output_topic", "/downsampled_cloud");
        this->get_parameter("output_topic", output_topic_);
        this->declare_parameter<bool>("front_view_only", true); // Default to true
        this->get_parameter("front_view_only", front_view_only_);

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&VoxelGridFilterNode::cloud_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "VoxelGrid Filter Node started. (Front = -X)");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Leaf size: %.3f", leaf_size_);
        RCLCPP_INFO(this->get_logger(), "Front view only: %s", front_view_only_ ? "true" : "false");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (front_view_only_) {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(-10.0, 0.0); // Front now defined as negative X
            pass.filter(*cloud_front_filtered);
        } else {
            *cloud_front_filtered = *cloud;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor; sor.setInputCloud(cloud_front_filtered); sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_); sor.filter(*cloud_voxel_filtered);

        sensor_msgs::msg::PointCloud2 output_msg; pcl::toROSMsg(*cloud_voxel_filtered, output_msg); output_msg.header = msg->header; publisher_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    double leaf_size_; std::string input_topic_; std::string output_topic_; bool front_view_only_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelGridFilterNode>());
    rclcpp::shutdown();
    return 0;
}