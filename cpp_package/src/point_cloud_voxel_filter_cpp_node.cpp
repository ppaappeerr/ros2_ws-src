#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudVoxelFilterCppNode : public rclcpp::Node
{
public:
    PointCloudVoxelFilterCppNode() : Node("point_cloud_voxel_filter_cpp_node")
    {
        // Parameters
        this->declare_parameter<double>("leaf_size", 0.05);
        this->get_parameter("leaf_size", leaf_size_);
        this->declare_parameter<std::string>("input_topic", "/accumulated_cloud_cpp");
        this->get_parameter("input_topic", input_topic_);

        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_filtered_cloud_cpp", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&PointCloudVoxelFilterCppNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ Voxel Filter started. Leaf size: %.3f m", leaf_size_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert ROS message to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->points.empty()) return;

        // Apply VoxelGrid filter
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        // Convert back to ROS message and publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = msg->header;
        publisher_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    double leaf_size_;
    std::string input_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudVoxelFilterCppNode>());
    rclcpp::shutdown();
    return 0;
}
