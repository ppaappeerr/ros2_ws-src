#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudStatisticalOutlierRemoverCppNode : public rclcpp::Node
{
public:
    PointCloudStatisticalOutlierRemoverCppNode() : Node("point_cloud_statistical_outlier_remover_cpp_node")
    {
        // Parameters
        this->declare_parameter<int>("mean_k", 50);
        this->declare_parameter<double>("std_dev_mul_thresh", 1.0);
        this->declare_parameter<std::string>("input_topic", "/passthrough_filtered_cloud_cpp");

        this->get_parameter("mean_k", mean_k_);
        this->get_parameter("std_dev_mul_thresh", std_dev_mul_thresh_);
        this->get_parameter("input_topic", input_topic_);

        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cleaned_cloud_cpp", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&PointCloudStatisticalOutlierRemoverCppNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ Statistical Outlier Remover started. MeanK: %d, StdDevThresh: %.2f", mean_k_, std_dev_mul_thresh_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->points.empty()) return;

        // Apply StatisticalOutlierRemoval filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(std_dev_mul_thresh_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.filter(*filtered_cloud);

        // Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = msg->header;
        publisher_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    int mean_k_;
    double std_dev_mul_thresh_;
    std::string input_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudStatisticalOutlierRemoverCppNode>());
    rclcpp::shutdown();
    return 0;
}
