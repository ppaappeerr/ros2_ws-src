#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <mutex>

class ZVarianceSweeperNode : public rclcpp::Node
{
public:
    ZVarianceSweeperNode() : Node("z_variance_sweeper_node")
    {
        // Parameters
        this->declare_parameter<double>("variance_threshold", 0.01); // Threshold for Z variance
        this->declare_parameter<int>("min_points_for_sweep", 1000);
        this->get_parameter("variance_threshold", variance_threshold_);
        this->get_parameter("min_points_for_sweep", min_points_for_sweep_);

        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/z_variance_sweep", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/dense_points", qos, std::bind(&ZVarianceSweeperNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Z-Variance Sweeper started. Threshold: %.4f", variance_threshold_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromROSMsg(*msg, temp_cloud);
        
        buffer_cloud_.points.insert(buffer_cloud_.points.end(), temp_cloud.points.begin(), temp_cloud.points.end());

        if (buffer_cloud_.points.size() < static_cast<size_t>(min_points_for_sweep_))
        {
            return; // Not enough points to calculate variance reliably
        }

        // Calculate Z-variance
        double sum_z = 0.0, sum_sq_z = 0.0;
        for (const auto& point : buffer_cloud_.points)
        {
            sum_z += point.z;
            sum_sq_z += point.z * point.z;
        }
        double mean_z = sum_z / buffer_cloud_.points.size();
        double variance_z = (sum_sq_z / buffer_cloud_.points.size()) - (mean_z * mean_z);

        // If variance exceeds threshold, publish and clear
        if (variance_z > variance_threshold_)
        {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(buffer_cloud_, output_msg);
            output_msg.header.stamp = this->get_clock()->now();
            output_msg.header.frame_id = "base_link";
            publisher_->publish(output_msg);

            RCLCPP_INFO(this->get_logger(), "Published a sweep with %zu points (Z-variance: %.4f)", buffer_cloud_.points.size(), variance_z);

            buffer_cloud_.clear();
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    double variance_threshold_;
    int min_points_for_sweep_;
    pcl::PointCloud<pcl::PointXYZ> buffer_cloud_;
    std::mutex mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZVarianceSweeperNode>());
    rclcpp::shutdown();
    return 0;
}
