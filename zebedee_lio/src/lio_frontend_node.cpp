#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "zebedee_lio/submap_manager.hpp"

#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Geometry>

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

class LioFrontendNode : public rclcpp::Node
{
public:
  LioFrontendNode() : Node("lio_frontend_node"), is_initialized_(false)
  {
    RCLCPP_INFO(this->get_logger(), "LIO Frontend Node Started.");

    // 파라미터 선언
    this->declare_parameter<double>("submap.sliding_window_size", 15.0);
    this->declare_parameter<double>("submap.voxel_leaf_size", 0.25);
    this->declare_parameter<double>("icp.voxel_leaf_size", 0.4);

    double sliding_window_size = this->get_parameter("submap.sliding_window_size").as_double();
    double submap_voxel_size = this->get_parameter("submap.voxel_leaf_size").as_double();
    double icp_voxel_size = this->get_parameter("icp.voxel_leaf_size").as_double();

    submap_manager_ = std::make_unique<zebedee_lio::SubmapManager>(sliding_window_size, submap_voxel_size);
    RCLCPP_INFO(this->get_logger(), "SubmapManager initialized.");

    // ICP 최적화를 위한 VoxelGrid 필터 초기화
    icp_voxel_filter_.setLeafSize(icp_voxel_size, icp_voxel_size, icp_voxel_size);
    RCLCPP_INFO(this->get_logger(), "ICP Voxel Filter initialized with leaf size: %.2f", icp_voxel_size);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/points_3d", 10, std::bind(&LioFrontendNode::pointcloudCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", qos, std::bind(&LioFrontendNode::imuCallback, this, std::placeholders::_1));

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/lio/odometry", 10);
    submap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio/submap", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    icp_.setMaxCorrespondenceDistance(1.0);
    icp_.setTransformationEpsilon(1e-6);
    icp_.setEuclideanFitnessEpsilon(1e-6);
    icp_.setMaximumIterations(30);

    current_pose_.setIdentity();
    last_imu_orientation_.setIdentity();
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_orientation_ = Eigen::Quaterniond(msg->orientation.w,
                                             msg->orientation.x,
                                             msg->orientation.y,
                                             msg->orientation.z);
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    PointCloud::Ptr current_cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *current_cloud);

    if (current_cloud->empty()) return;

    // 입력 스캔 다운샘플링
    PointCloud::Ptr filtered_cloud(new PointCloud());
    icp_voxel_filter_.setInputCloud(current_cloud);
    icp_voxel_filter_.filter(*filtered_cloud);

    if (!is_initialized_)
    {
      submap_manager_->addPointCloud(filtered_cloud);
      is_initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "System Initialized.");
      publishSubmap(msg->header);
      return;
    }

    PointCloud::Ptr submap = submap_manager_->getSubmap();
    if (submap->size() < 10) {
        submap_manager_->addPointCloud(filtered_cloud);
        return;
    }

    // ICP의 Target이 될 서브맵도 다운샘플링
    PointCloud::Ptr submap_downsampled(new PointCloud());
    icp_voxel_filter_.setInputCloud(submap);
    icp_voxel_filter_.filter(*submap_downsampled);

    // ICP 정합 실행
    icp_.setInputSource(filtered_cloud);
    icp_.setInputTarget(submap_downsampled);

    Eigen::Matrix4f initial_guess = current_pose_.cast<float>();
    PointCloud::Ptr aligned_cloud(new PointCloud());
    icp_.align(*aligned_cloud, initial_guess);

    if (!icp_.hasConverged())
    {
      RCLCPP_WARN(this->get_logger(), "ICP failed to converge.");
      return;
    }

    // 결과 누적 및 맵 업데이트
    current_pose_ = icp_.getFinalTransformation().cast<double>();
    submap_manager_->addPointCloud(aligned_cloud);
    submap_manager_->updateSlidingWindow(current_pose_.block<3,1>(0,3));

    publishOdometryAndTF(msg->header);
    publishSubmap(msg->header);
  }

  void publishOdometryAndTF(const std_msgs::msg::Header& header)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    Eigen::Matrix3d rotation_matrix = current_pose_.block<3, 3>(0, 0);
    Eigen::Quaterniond q(rotation_matrix);
    q.normalize();

    odom_msg.pose.pose.position.x = current_pose_(0, 3);
    odom_msg.pose.pose.position.y = current_pose_(1, 3);
    odom_msg.pose.pose.position.z = current_pose_(2, 3);
    odom_msg.pose.pose.orientation.w = q.w();
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odometry_pub_->publish(odom_msg);

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
  pcl::VoxelGrid<PointType> icp_voxel_filter_;
  
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