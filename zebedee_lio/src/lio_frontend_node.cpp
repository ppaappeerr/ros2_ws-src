#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "zebedee_lio/submap_manager.hpp" // 우리가 만든 SubmapManager

#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

using namespace std::chrono_literals;

class LioFrontendNode : public rclcpp::Node
{
public:
  LioFrontendNode() : Node("lio_frontend_node"), is_initialized_(false)
  {
    RCLCPP_INFO(this->get_logger(), "LIO Frontend Node Started.");

    // 파라미터 선언 및 초기화
    this->declare_parameter<double>("submap.sliding_window_size", 10.0);
    this->declare_parameter<double>("submap.voxel_leaf_size", 0.2);
    double sliding_window_size = this->get_parameter("submap.sliding_window_size").as_double();
    double voxel_leaf_size = this->get_parameter("submap.voxel_leaf_size").as_double();

    // SubmapManager 인스턴스 생성
    submap_manager_ = std::make_unique<zebedee_lio::SubmapManager>(sliding_window_size, voxel_leaf_size);
    RCLCPP_INFO(this->get_logger(), "SubmapManager initialized with window size: %.2f, voxel size: %.2f",
                sliding_window_size, voxel_leaf_size);


    // QoS 설정
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    // 구독자 (Subscriber) 설정
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_3d", 10, std::bind(&LioFrontendNode::pointcloudCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos, std::bind(&LioFrontendNode::imuCallback, this, std::placeholders::_1));

    // 발행자 (Publisher) 설정
    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lio/odometry", 10);
    submap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/submap", 10);

    // TF Broadcaster 설정
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ICP 설정
    icp_.setMaxCorrespondenceDistance(1.0);
    icp_.setTransformationEpsilon(1e-6);
    icp_.setEuclideanFitnessEpsilon(1e-6);
    icp_.setMaximumIterations(30);

    // 현재 위치 및 자세 초기화
    current_pose_.setIdentity();
    last_imu_orientation_.setIdentity();
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 간단한 IMU 자세 예측 (나중에 Pre-integration으로 고도화 예정)
    // 지금은 들어온 IMU의 orientation을 그대로 사용
    last_imu_orientation_ = Eigen::Quaterniond(msg->orientation.w,
                                             msg->orientation.x,
                                             msg->orientation.y,
                                             msg->orientation.z);
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloud::Ptr current_cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *current_cloud);

    if (current_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
      return;
    }

    // 시스템 초기화
    if (!is_initialized_)
    {
      // 첫 스캔은 서브맵에 추가만 하고 위치 추정은 건너뜀
      submap_manager_->addPointCloud(current_cloud);
      is_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "System Initialized with first scan.");
      return;
    }

    // ICP를 위한 준비
    PointCloud::Ptr submap = submap_manager_->getSubmap();
    if (submap->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Submap has too few points. Skipping frame.");
      submap_manager_->addPointCloud(current_cloud);
      return;
    }
    
    icp_.setInputTarget(submap);
    icp_.setInputSource(current_cloud);

    // IMU 데이터로 초기 추정치 계산
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess.block<3, 3>(0, 0) = last_imu_orientation_.toRotationMatrix().cast<float>();
    // 위치는 이전 위치를 그대로 사용 (Matrix4d에서 translation 벡터 추출)
    initial_guess.block<3, 1>(0, 3) = current_pose_.block<3, 1>(0, 3).cast<float>();

    // ICP 정합 실행
    PointCloud::Ptr aligned_cloud(new PointCloud());
    icp_.align(*aligned_cloud, initial_guess);

    if (!icp_.hasConverged())
    {
      RCLCPP_WARN(this->get_logger(), "ICP failed to converge. Fitness score: %f", icp_.getFitnessScore());
      // 수렴 실패 시 현재 스캔은 버림
      return;
    }

    // 결과 업데이트
    Eigen::Matrix4d transformation = icp_.getFinalTransformation().cast<double>();
    current_pose_ = transformation;

    // 새로운 스캔을 서브맵에 추가 및 슬라이딩 윈도우 업데이트
    PointCloud::Ptr transformed_cloud(new PointCloud());
    pcl::transformPointCloud(*current_cloud, *transformed_cloud, transformation);
    submap_manager_->addPointCloud(transformed_cloud);
    // Matrix4d에서 translation 벡터 추출
    Eigen::Vector3d current_position = current_pose_.block<3, 1>(0, 3);
    submap_manager_->updateSlidingWindow(current_position);

    // Odometry 및 TF 발행
    publishOdometryAndTF(msg->header);

    // 시각화를 위해 현재 서브맵 발행
    publishSubmap(msg->header);
  }

  void publishOdometryAndTF(const std_msgs::msg::Header& header)
  {
    // Odometry 메시지 채우기
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Matrix4d에서 회전 행렬 추출하고 Quaternion으로 변환
    Eigen::Matrix3d rotation_matrix = current_pose_.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation_matrix);
    q.normalize();

    // 위치 정보 추출
    odom_msg.pose.pose.position.x = current_pose_(0, 3);
    odom_msg.pose.pose.position.y = current_pose_(1, 3);
    odom_msg.pose.pose.position.z = current_pose_(2, 3);
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odometry_pub_->publish(odom_msg);

    // TF 메시지 채우기 (odom -> base_link)
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header = odom_msg.header;
    tf_stamped.child_frame_id = odom_msg.child_frame_id;
    tf_stamped.transform.translation.x = odom_msg.pose.pose.position.x;
    tf_stamped.transform.translation.y = odom_msg.pose.pose.position.y;
    tf_stamped.transform.translation.z = odom_msg.pose.pose.position.z;
    tf_stamped.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_stamped);
  }

  void publishSubmap(const std_msgs::msg::Header& header)
  {
      sensor_msgs::msg::PointCloud2 submap_msg;
      pcl::toROSMsg(*(submap_manager_->getSubmap()), submap_msg);
      submap_msg.header.stamp = header.stamp;
      submap_msg.header.frame_id = "odom";
      submap_pub_->publish(submap_msg);
  }

  // 멤버 변수
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<zebedee_lio::SubmapManager> submap_manager_;
  pcl::IterativeClosestPoint<PointType, PointType> icp_;
  
  bool is_initialized_;
  Eigen::Matrix4d current_pose_;
  Eigen::Quaterniond last_imu_orientation_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LioFrontendNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}