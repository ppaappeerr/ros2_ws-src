#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class PointCloudPassThroughFilterCppNode : public rclcpp::Node
{
public:
    PointCloudPassThroughFilterCppNode() : Node("point_cloud_passthrough_filter_cpp_node")
    {
        // Parameters
        this->declare_parameter<double>("min_limit", -1.5);
        this->declare_parameter<double>("max_limit", 0.5);
        this->declare_parameter<std::string>("input_topic", "/sweep_cloud_cpp");
        
        this->get_parameter("min_limit", min_limit_);
        this->get_parameter("max_limit", max_limit_);
        this->get_parameter("input_topic", input_topic_);

        // QoS Profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        // Publisher and Subscriber
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/passthrough_filtered_cloud_cpp", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos, std::bind(&PointCloudPassThroughFilterCppNode::topic_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "C++ PassThrough Filter started. Z-axis limits: [%.2f, %.2f]", min_limit_, max_limit_);
    }

private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->points.empty()) return;

        // Apply PassThrough filter
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(input_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(min_limit_, max_limit_);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(*filtered_cloud);

        // Publish
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = msg->header;
        publisher_->publish(output_msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    
    double min_limit_, max_limit_;
    std::string input_topic_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPassThroughFilterCppNode>());
    rclcpp::shutdown();
    return 0;
}
