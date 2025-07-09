#include <iostream>
#include <memory>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h> // PassThrough 필터 헤더 추가

class ProjectedGroundDetector : public rclcpp::Node
{
public:
    ProjectedGroundDetector()
        : Node("projected_ground_detector_node")
    {
        // QoS 설정
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // 구독자 및 발행자 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sweep_cloud_cpp", qos, std::bind(&ProjectedGroundDetector::cloud_callback, this, std::placeholders::_1));
        
        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/projected_ground_points", 10);
        obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/projected_obstacle_points", 10);

        // RANSAC 파라미터 선언
        this->declare_parameter<double>("ransac_distance_threshold", 0.03);
        
        // 평면 모델 초기화
        plane_model_ = std::make_shared<pcl::ModelCoefficients>();

        RCLCPP_INFO(this->get_logger(), "Projected Ground Detector Node has been started.");
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

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (!is_plane_found_)
        {
            // 최초 평면 탐색
            find_initial_plane(input_cloud);
        }
        else
        {
            // 기존 평면 기준으로 점 분리 및 투영
            separate_and_project_points(input_cloud, ground_cloud, obstacle_cloud);
            
            if (!ground_cloud->points.empty())
            {
                // 격자 형태의 시각화용 클라우드 생성
                pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud = create_grid_visualization();
                
                sensor_msgs::msg::PointCloud2 ground_msg;
                pcl::toROSMsg(*grid_cloud, ground_msg);
                ground_msg.header = msg->header;
                ground_publisher_->publish(ground_msg);
            }
        }

        // 장애물 포인트 클라우드 발행
        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        obstacle_msg.header = msg->header;
        obstacle_publisher_->publish(obstacle_msg);
    }

    void find_initial_plane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        // 1. 사전 필터링: 바닥일 가능성이 있는 영역의 점들만 추출
        pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1.7, -1.0); // 센서 높이를 고려하여 바닥 예상 영역만 필터링 (값 조절 필요)
        pass.filter(*candidate_points);

        if (candidate_points->points.size() < 10) { // 최소 포인트 수
             RCLCPP_WARN(this->get_logger(), "Not enough candidate points to find initial plane. Waiting for more data.");
             return;
        }

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(this->get_parameter("ransac_distance_threshold").as_double());
        seg.setInputCloud(candidate_points); // 필터링된 점들로 RANSAC 수행
        seg.segment(*inliers, *plane_model_);

        if (inliers->indices.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model. Trying again.");
            return;
        }

        // 2. 평면 품질 검사: 법선 벡터가 Z축 방향에 가까운지 확인
        double normal_z = std::abs(plane_model_->values[2]);
        if (normal_z < 0.9) { // 수평면에 가깝지 않으면 무시 (임계값 0.9는 약 25도 이내의 기울기)
            RCLCPP_WARN(this->get_logger(), "Found a plane, but it's not horizontal enough (normal_z: %f). Retrying.", normal_z);
            return; // 아직 올바른 평면을 찾지 못했으므로 is_plane_found_를 true로 바꾸지 않음
        }

        is_plane_found_ = true;
        RCLCPP_INFO(this->get_logger(), "Initial HORIZONTAL ground plane found!");
        RCLCPP_INFO(this->get_logger(), "Model coefficients: %f, %f, %f, %f",
                    plane_model_->values[0], plane_model_->values[1],
                    plane_model_->values[2], plane_model_->values[3]);
    }

    void separate_and_project_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& ground_points,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_points)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr projected_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(cloud);
        proj.setModelCoefficients(plane_model_);
        proj.filter(*projected_points);

        double dist_thresh = this->get_parameter("ransac_distance_threshold").as_double();

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto& pt = cloud->points[i];
            const auto& proj_pt = projected_points->points[i];
            double dist = std::sqrt(std::pow(pt.x - proj_pt.x, 2) +
                                    std::pow(pt.y - proj_pt.y, 2) +
                                    std::pow(pt.z - proj_pt.z, 2));

            if (dist < dist_thresh)
            {
                ground_points->points.push_back(proj_pt); // 평면에 투영된 점을 추가
            }
            else
            {
                obstacle_points->points.push_back(pt);
            }
        }
        
        // 누적된 ground_cloud_에 새로운 지면 포인트 추가 및 VoxelGrid 필터링
        *accumulated_ground_cloud_ += *ground_points;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(accumulated_ground_cloud_);
        sor.setLeafSize(0.05f, 0.05f, 0.05f);
        sor.filter(*accumulated_ground_cloud_);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr create_grid_visualization()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (accumulated_ground_cloud_->points.empty()) return grid_cloud;

        // 경계 찾기
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*accumulated_ground_cloud_, min_pt, max_pt);
        
        float grid_resolution = 0.2f; // 20cm 간격의 격자

        // 격자 생성
        for (float x = min_pt.x; x <= max_pt.x; x += grid_resolution)
        {
            for (float y = min_pt.y; y <= max_pt.y; y += grid_resolution)
            {
                // 평면 방정식 ax + by + cz + d = 0, 여기서 z를 계산
                // z = (-ax - by - d) / c
                if (std::abs(plane_model_->values[2]) > 1e-6) {
                    float z = (-plane_model_->values[0] * x - plane_model_->values[1] * y - plane_model_->values[3]) / plane_model_->values[2];
                    grid_cloud->points.emplace_back(x, y, z);
                }
            }
        }
        grid_cloud->width = grid_cloud->points.size();
        grid_cloud->height = 1;
        grid_cloud->is_dense = true;

        return grid_cloud;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_ground_cloud_{new pcl::PointCloud<pcl::PointXYZ>};
    pcl::ModelCoefficients::Ptr plane_model_;
    bool is_plane_found_ = false;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ProjectedGroundDetector>());
    rclcpp::shutdown();
    return 0;
}