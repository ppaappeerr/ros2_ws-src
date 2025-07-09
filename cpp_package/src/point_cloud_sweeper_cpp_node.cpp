#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>

class PointCloudSweeperCppNode : public rclcpp::Node
{
public:
    PointCloudSweeperCppNode() : Node("point_cloud_sweeper_cpp_node")
    {
        // Parameters
        this->declare_parameter<double>("window_size_seconds", 1.5);
        this->get_parameter("window_size_seconds", window_size_);

        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sweep_cloud_cpp", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dense_points", qos, std::bind(&PointCloudSweeperCppNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ PointCloud Sweeper started. Window size: %.2f s", window_size_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // 1. Convert ROS message to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *current_cloud);

        // 2. Add new cloud with its timestamp to the deque
        double current_timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        ring_buffer_.push_back({current_timestamp, current_cloud});

        // 3. Remove old clouds from the front of the deque
        while (!ring_buffer_.empty() && (current_timestamp - ring_buffer_.front().first > window_size_))
        {
            ring_buffer_.pop_front();
        }

        // 4. Concatenate all clouds in the buffer
        pcl::PointCloud<pcl::PointXYZ>::Ptr sweep_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& pair : ring_buffer_)
        {
            *sweep_cloud += *pair.second;
        }

        // 5. Convert concatenated PCL cloud back to ROS message and publish
        if (!sweep_cloud->points.empty())
        {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*sweep_cloud, output_msg);
            output_msg.header = msg->header; // Use the latest timestamp
            output_msg.header.frame_id = "base_link";
            publisher_->publish(output_msg);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    double window_size_;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZ>::Ptr>> ring_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudSweeperCppNode>());
    rclcpp::shutdown();
    return 0;
}
