#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <vector>
#include <chrono>

class ICP3DOdomNode : public rclcpp::Node {
private:
    // ROS2 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // PCL 포인트클라우드
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    using PointCloudPtr = PointCloud::Ptr;
    
    PointCloudPtr prev_cloud_;
    Eigen::Matrix4f current_pose_;
    
    // ICP 파라미터
    double max_correspondence_distance_;
    int max_iterations_;
    double transformation_epsilon_;
    double euclidean_fitness_epsilon_;
    double voxel_size_;
    bool publish_tf_;
    
    // 성능 최적화 추가
    double max_processing_time_;
    size_t min_points_threshold_;
    size_t max_points_threshold_;
    bool adaptive_voxel_size_;
    double base_voxel_size_;

    // 시간 추적
    rclcpp::Time last_time_;
    bool first_scan_;
    
    // 성능 통계
    size_t scan_count_;
    double total_icp_time_;

public:
    ICP3DOdomNode() : Node("icp_3d_odom_cpp") {
        // 파라미터 선언
        this->declare_parameter("input_topic", "/formatted_cloud");
        this->declare_parameter("output_topic", "/lio_odom");
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("max_iterations", 50);
        this->declare_parameter("transformation_epsilon", 1e-8);
        this->declare_parameter("euclidean_fitness_epsilon", 1e-6);
        this->declare_parameter("voxel_size", 0.05);
        this->declare_parameter("publish_tf", true);
        
        // 성능 파라미터 추가
        this->declare_parameter("max_processing_time", 30.0);  // 30ms 제한
        this->declare_parameter("min_points_threshold", 50);
        this->declare_parameter("max_points_threshold", 2000);
        this->declare_parameter("adaptive_voxel_size", true);
        
        // 파라미터 읽기
        max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
        max_iterations_ = this->get_parameter("max_iterations").as_int();
        transformation_epsilon_ = this->get_parameter("transformation_epsilon").as_double();
        euclidean_fitness_epsilon_ = this->get_parameter("euclidean_fitness_epsilon").as_double();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        
        max_processing_time_ = this->get_parameter("max_processing_time").as_double();
        min_points_threshold_ = this->get_parameter("min_points_threshold").as_int();
        max_points_threshold_ = this->get_parameter("max_points_threshold").as_int();
        adaptive_voxel_size_ = this->get_parameter("adaptive_voxel_size").as_bool();
        base_voxel_size_ = voxel_size_;
        
        // 초기화
        current_pose_ = Eigen::Matrix4f::Identity();
        first_scan_ = true;
        scan_count_ = 0;
        total_icp_time_ = 0.0;
        
        // 구독자/발행자 생성
        pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("input_topic").as_string(), 10,
            std::bind(&ICP3DOdomNode::pointCloudCallback, this, std::placeholders::_1));
            
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("output_topic").as_string(), 10);
        
        if (publish_tf_) {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
        
        RCLCPP_INFO(this->get_logger(), "ICP 3D Odometry Node (C++ PCL) started");
        RCLCPP_INFO(this->get_logger(), "Input: %s -> Output: %s", 
                   this->get_parameter("input_topic").as_string().c_str(),
                   this->get_parameter("output_topic").as_string().c_str());
        RCLCPP_INFO(this->get_logger(), "ICP params: max_dist=%.3f, max_iter=%d, voxel=%.3f", 
                   max_correspondence_distance_, max_iterations_, voxel_size_);
        RCLCPP_INFO(this->get_logger(), "Performance params: max_time=%.1fms, points=[%zu-%zu]", 
                   max_processing_time_, min_points_threshold_, max_points_threshold_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // ROS2 PointCloud2를 PCL로 변환
        PointCloudPtr current_cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *current_cloud);
        
        if (current_cloud->empty()) {
            return;
        }
        
        // 🚀 최적화 1: 빠른 전처리
        PointCloudPtr current_cloud_processed = preprocessCloud(current_cloud);
        
        if (current_cloud_processed->size() < min_points_threshold_) {
            RCLCPP_DEBUG(this->get_logger(), "Too few points: %zu", current_cloud_processed->size());
            return;
        }
        
        if (first_scan_) {
            prev_cloud_ = current_cloud_processed;
            last_time_ = this->get_clock()->now();
            first_scan_ = false;
            RCLCPP_INFO(this->get_logger(), "First scan: %zu points", current_cloud_processed->size());
            return;
        }
        
        // 🚀 최적화 2: 시간 제한 ICP
        if (prev_cloud_ && !prev_cloud_->empty()) {
            auto icp_start = std::chrono::high_resolution_clock::now();
            
            auto [transformation, fitness, converged] = performFastICP(prev_cloud_, current_cloud_processed);
            
            auto icp_end = std::chrono::high_resolution_clock::now();
            double icp_time = std::chrono::duration<double, std::milli>(icp_end - icp_start).count();
            
            // 🚀 최적화 3: 적응적 품질 체크
            double fitness_threshold = adaptive_voxel_size_ ? 
                std::min(0.3, 0.1 + current_cloud_processed->size() / 10000.0) : 0.2;
            
            if (converged && fitness < fitness_threshold) {
                current_pose_ = current_pose_ * transformation;
                publishOdometry(msg->header, current_pose_);
                
                if (publish_tf_) {
                    publishTransform(msg->header, current_pose_);
                }
                
                scan_count_++;
                total_icp_time_ += icp_time;
                
                RCLCPP_DEBUG(this->get_logger(), 
                           "ICP OK: fitness=%.3f, time=%.1fms, pts=%zu", 
                           fitness, icp_time, current_cloud_processed->size());
            } else {
                RCLCPP_DEBUG(this->get_logger(), "ICP skip: fitness=%.3f", fitness);
            }
        }
        
        prev_cloud_ = current_cloud_processed;
        
        auto end_time = std::chrono::high_resolution_clock::now();
        double total_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        // 경고 임계값 상향 조정
        if (total_time > max_processing_time_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,  // 2초마다만 경고
                                "Slow processing: %.1fms (points: %zu)", 
                                total_time, current_cloud_processed->size());
        }
    }
    
    // 🚀 최적화된 전처리 함수
    PointCloudPtr preprocessCloud(const PointCloudPtr& cloud) {
        PointCloudPtr cloud_processed(new PointCloud);
        
        // 1. 빠른 유효성 체크 + 범위 필터링
        cloud_processed->reserve(cloud->size());
        for (const auto& point : cloud->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                // 더 엄격한 범위 체크 (속도 향상)
                if (std::abs(point.x) < 8.0 && std::abs(point.y) < 8.0 && 
                    point.z > -2.0 && point.z < 3.0) {
                    cloud_processed->push_back(point);
                }
            }
        }
        
        // 2. 적응적 다운샘플링
        if (cloud_processed->size() > max_points_threshold_) {
            // 포인트 수가 많으면 voxel 크기 증가
            double adaptive_voxel = adaptive_voxel_size_ ? 
                base_voxel_size_ * (1.0 + cloud_processed->size() / 5000.0) : base_voxel_size_;
            
            PointCloudPtr cloud_downsampled(new PointCloud);
            pcl::VoxelGrid<PointType> voxel_filter;
            voxel_filter.setInputCloud(cloud_processed);
            voxel_filter.setLeafSize(adaptive_voxel, adaptive_voxel, adaptive_voxel);
            voxel_filter.filter(*cloud_downsampled);
            
            return cloud_downsampled;
        }
        
        return cloud_processed;
    }
    
    // 🚀 최적화된 ICP 함수
    std::tuple<Eigen::Matrix4f, double, bool> performFastICP(const PointCloudPtr& source, 
                                                             const PointCloudPtr& target) {
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setInputSource(source);
        icp.setInputTarget(target);
        
        // 적응적 파라미터 조정
        size_t total_points = source->size() + target->size();
        
        if (total_points > 2000) {
            // 포인트 많으면 → 빠른 설정
            icp.setMaxCorrespondenceDistance(max_correspondence_distance_ * 1.5);
            icp.setMaximumIterations(std::max(20, max_iterations_ / 2));
        } else {
            // 포인트 적으면 → 정확한 설정
            icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
            icp.setMaximumIterations(max_iterations_);
        }
        
        icp.setTransformationEpsilon(transformation_epsilon_);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
        
        PointCloud result;
        icp.align(result);
        
        return std::make_tuple(
            icp.getFinalTransformation(),
            icp.getFitnessScore(),
            icp.hasConverged()
        );
    }
    
    void publishOdometry(const std_msgs::msg::Header& header, const Eigen::Matrix4f& pose) {
        auto odom_msg = nav_msgs::msg::Odometry();
        
        // 헤더 설정
        odom_msg.header = header;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // 위치 설정
        odom_msg.pose.pose.position.x = pose(0, 3);
        odom_msg.pose.pose.position.y = pose(1, 3);
        odom_msg.pose.pose.position.z = pose(2, 3);
        
        // 회전 설정 (회전 행렬을 쿼터니언으로 변환)
        Eigen::Matrix3f rotation = pose.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation);
        q.normalize();
        
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // 공분산 설정 (기본값)
        std::array<double, 36> pose_cov = {0};
        pose_cov[0] = 0.01;   // x
        pose_cov[7] = 0.01;   // y  
        pose_cov[14] = 0.01;  // z
        pose_cov[21] = 0.01;  // roll
        pose_cov[28] = 0.01;  // pitch
        pose_cov[35] = 0.01;  // yaw
        odom_msg.pose.covariance = pose_cov;
        
        std::array<double, 36> twist_cov = {0};
        twist_cov[0] = 0.1;   // linear x
        twist_cov[35] = 0.1;  // angular z
        odom_msg.twist.covariance = twist_cov;
        
        odom_pub_->publish(odom_msg);
    }
    
    void publishTransform(const std_msgs::msg::Header& header, const Eigen::Matrix4f& pose) {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header = header;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        // 위치
        transform.transform.translation.x = pose(0, 3);
        transform.transform.translation.y = pose(1, 3);
        transform.transform.translation.z = pose(2, 3);
        
        // 회전
        Eigen::Matrix3f rotation = pose.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation);
        q.normalize();
        
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ICP3DOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}