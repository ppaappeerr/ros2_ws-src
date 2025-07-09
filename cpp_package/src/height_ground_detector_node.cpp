#include <iostream>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

class HeightGroundDetector : public rclcpp::Node
{
public:
    HeightGroundDetector()
        : Node("height_ground_detector_node")
    {
        // QoS 설정
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 파라미터 선언 (ROS 2 파라미터로 실시간 조절 가능)
        this->declare_parameter<double>("ground_height_min", -1.6);
        this->declare_parameter<double>("ground_height_max", -1.4);
        this->declare_parameter<double>("voxel_leaf_size", 0.05);

        // 구독자 및 발행자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sweep_cloud_cpp", qos, std::bind(&HeightGroundDetector::cloud_callback, this, std::placeholders::_1));
        
        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/stable_ground_points", 10);
        obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", 10);

        RCLCPP_INFO(this->get_logger(), "Height Ground Detector Node has been started.");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->points.empty())
        {
            return;
        }

        // 지면과 장애물 분리를 위한 임시 클라우드
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_ground_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 파라미터에서 높이 값 가져오기
        double min_z = this->get_parameter("ground_height_min").as_double();
        double max_z = this->get_parameter("ground_height_max").as_double();

        // Z축 높이 기준으로 지면/장애물 분리
        for (const auto& point : input_cloud->points)
        {
            if (point.z >= min_z && point.z <= max_z)
            {
                current_ground_points->points.push_back(point);
            }
            else
            {
                obstacle_cloud->points.push_back(point);
            }
        }

        // 누적 지면 클라우드에 현재 지면 포인트 추가
        *accumulated_ground_cloud_ += *current_ground_points;

        // VoxelGrid 필터를 사용하여 누적 클라우드의 밀도 조절
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(accumulated_ground_cloud_);
        sor.setLeafSize(
            this->get_parameter("voxel_leaf_size").as_double(),
            this->get_parameter("voxel_leaf_size").as_double(),
            this->get_parameter("voxel_leaf_size").as_double()
        );
        sor.filter(*downsampled_cloud);
        
        // 필터링된 클라우드를 다시 누적 클라우드에 할당
        accumulated_ground_cloud_ = downsampled_cloud;

        // 메시지 발행
        sensor_msgs::msg::PointCloud2 ground_msg;
        pcl::toROSMsg(*accumulated_ground_cloud_, ground_msg);
        ground_msg.header = msg->header;
        ground_publisher_->publish(ground_msg);

        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        obstacle_msg.header = msg->header;
        obstacle_publisher_->publish(obstacle_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_ground_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightGroundDetector>());
    rclcpp::shutdown();
    return 0;
}