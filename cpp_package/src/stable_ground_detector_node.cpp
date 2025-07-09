#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class StableGroundDetectorNode : public rclcpp::Node
{
public:
    StableGroundDetectorNode() : Node("stable_ground_detector_node")
    {
        // Parameters
        this->declare_parameter<std::string>("input_topic", "/sweep_cloud_cpp");
        this->declare_parameter<std::string>("ground_topic", "/stable_ground_points");
        this->declare_parameter<std::string>("obstacle_topic", "/obstacle_points");
        this->declare_parameter<double>("ground_height_threshold_min", -1.7);
        this->declare_parameter<double>("ground_height_threshold_max", -1.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.05);

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("ground_topic", ground_topic_);
        this->get_parameter("obstacle_topic", obstacle_topic_);
        this->get_parameter("ground_height_threshold_min", ground_height_threshold_min_);
        this->get_parameter("ground_height_threshold_max", ground_height_threshold_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

        RCLCPP_INFO(this->get_logger(), "Stable Ground Detector Node started.");
        RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Ground height range: [%.2f, %.2f]", ground_height_threshold_min_, ground_height_threshold_max_);

        // Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&StableGroundDetectorNode::cloud_callback, this, std::placeholders::_1));

        // Publishers
        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(ground_topic_, 10);
        obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);

        // Initialize accumulated ground cloud
        accumulated_ground_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty())
        {
            return;
        }

        // --- Obstacle Extraction ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_obs;
        pass_obs.setInputCloud(input_cloud);
        pass_obs.setFilterFieldName("z");
        pass_obs.setFilterLimits(ground_height_threshold_min_, ground_height_threshold_max_);
        pass_obs.setNegative(true); // Keep points *outside* the specified range
        pass_obs.filter(*obstacle_cloud);

        // --- Ground Candidate Extraction ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_ground;
        pass_ground.setInputCloud(input_cloud);
        pass_ground.setFilterFieldName("z");
        pass_ground.setFilterLimits(ground_height_threshold_min_, ground_height_threshold_max_);
        pass_ground.setNegative(false); // Keep points *inside* the specified range
        pass_ground.filter(*ground_candidates);

        // --- Accumulate Ground Points ---
        if (!ground_candidates->empty())
        {
            *accumulated_ground_cloud_ += *ground_candidates;
        }

        // --- Downsample Accumulated Ground Cloud ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!accumulated_ground_cloud_->empty())
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(accumulated_ground_cloud_);
            voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_grid.filter(*downsampled_ground_cloud);
        }

        // --- Publish topics ---
        sensor_msgs::msg::PointCloud2 ground_msg;
        if (!downsampled_ground_cloud->empty())
        {
            pcl::toROSMsg(*downsampled_ground_cloud, ground_msg);
            ground_msg.header = msg->header;
            ground_publisher_->publish(ground_msg);
        }

        sensor_msgs::msg::PointCloud2 obstacle_msg;
        if (!obstacle_cloud->empty())
        {
            pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
            obstacle_msg.header = msg->header;
            obstacle_publisher_->publish(obstacle_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_ground_cloud_;

    std::string input_topic_;
    std::string ground_topic_;
    std::string obstacle_topic_;
    double ground_height_threshold_min_;
    double ground_height_threshold_max_;
    double voxel_leaf_size_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StableGroundDetectorNode>());
    rclcpp::shutdown();
    return 0;
}