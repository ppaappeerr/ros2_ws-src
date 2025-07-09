#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudAccumulatorCppNode : public rclcpp::Node
{
public:
    PointCloudAccumulatorCppNode() : Node("point_cloud_accumulator_cpp_node")
    {
        // Parameters
        this->declare_parameter<double>("publish_hz", 2.0);
        double publish_hz = this->get_parameter("publish_hz").get_parameter_value().get<double>();
        
        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/accumulated_cloud_cpp", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dense_points", qos, std::bind(&PointCloudAccumulatorCppNode::topic_callback, this, std::placeholders::_1));
        
        // Timer for periodic publication
        auto period = std::chrono::duration<double>(1.0 / publish_hz);
        timer_ = this->create_wall_timer(period, std::bind(&PointCloudAccumulatorCppNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "C++ PointCloud Accumulator started. Publishing at %.2f Hz", publish_hz);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Add points from the incoming message to the buffer
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_cloud_.points.insert(buffer_cloud_.points.end(), temp_cloud.points.begin(), temp_cloud.points.end());
    }

    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_cloud_.points.empty())
        {
            return;
        }

        // Publish the whole buffer
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(buffer_cloud_, output_msg);
        output_msg.header.stamp = this->get_clock()->now();
        output_msg.header.frame_id = "base_link";
        publisher_->publish(output_msg);

        RCLCPP_INFO(this->get_logger(), "Published accumulated cloud with %zu points.", buffer_cloud_.points.size());

        // Clear the buffer for the next cycle
        buffer_cloud_.clear();
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    pcl::PointCloud<pcl::PointXYZ> buffer_cloud_;
    std::mutex mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudAccumulatorCppNode>());
    rclcpp::shutdown();
    return 0;
}
