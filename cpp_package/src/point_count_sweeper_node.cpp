#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

class PointCountSweeperNode : public rclcpp::Node
{
public:
    PointCountSweeperNode() : Node("point_count_sweeper_node")
    {
        // Parameters
        this->declare_parameter<int>("points_per_sweep", 5000);
        this->get_parameter("points_per_sweep", points_per_sweep_);

        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_count_sweep", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dense_points", qos, std::bind(&PointCountSweeperNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Point Count Sweeper started. Sweeping every %d points.", points_per_sweep_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        
        buffer_cloud_.points.insert(buffer_cloud_.points.end(), temp_cloud.points.begin(), temp_cloud.points.end());

        if (buffer_cloud_.points.size() >= static_cast<size_t>(points_per_sweep_))
        {
            sensor_msgs::msg::PointCloud2 output_msg;
            // Ensure we only publish the requested number of points
            pcl::PointCloud<pcl::PointXYZ> publish_cloud;
            publish_cloud.points.assign(buffer_cloud_.points.begin(), buffer_cloud_.points.begin() + points_per_sweep_);
            
            pcl::toROSMsg(publish_cloud, output_msg);
            output_msg.header.stamp = this->get_clock()->now();
            output_msg.header.frame_id = "base_link";
            publisher_->publish(output_msg);

            RCLCPP_INFO(this->get_logger(), "Published a sweep with %d points.", points_per_sweep_);

            // Remove the published points from the buffer
            buffer_cloud_.points.erase(buffer_cloud_.points.begin(), buffer_cloud_.points.begin() + points_per_sweep_);
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    int points_per_sweep_;
    pcl::PointCloud<pcl::PointXYZ> buffer_cloud_;
    std::mutex mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCountSweeperNode>());
    rclcpp::shutdown();
    return 0;
}
