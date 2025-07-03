#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <deque>
#include <chrono>

class IcpOdom : public rclcpp::Node
{
public:
  IcpOdom() : Node("icp_odometry")
  {
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/deskewed_cloud", 10,
        std::bind(&IcpOdom::cbCloud, this, std::placeholders::_1));
    sub_q_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/imu/attitude", 50,
        std::bind(&IcpOdom::cbQuat, this, std::placeholders::_1));
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/lidar_odom", 20);
    
    // 파라미터 설정
    this->declare_parameter("max_correspondence_distance", 0.15);
    this->declare_parameter("max_iterations", 80);
    this->declare_parameter("voxel_size", 0.05);
    this->declare_parameter("keyframe_dist", 0.10);
    this->declare_parameter("keyframe_angle", 0.03);
    
    max_corr_dist_ = this->get_parameter("max_correspondence_distance").as_double();
    max_iter_ = this->get_parameter("max_iterations").as_int();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    kf_dist_ = this->get_parameter("keyframe_dist").as_double();
    kf_angle_ = this->get_parameter("keyframe_angle").as_double();
    
    RCLCPP_INFO(this->get_logger(), "ICP Odometry with keyframe submap started");
  }

private:
  // 키프레임 관리
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> keyframes_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr submap_{new pcl::PointCloud<pcl::PointXYZI>()};
  
  // 상태 변수
  bool first_ = true;
  Eigen::Isometry3d T_world_curr_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d last_kf_pose_ = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond last_q_attitude_ = Eigen::Quaterniond::Identity();
  rclcpp::Time last_scan_time_;
  
  // 파라미터
  double max_corr_dist_, voxel_size_, kf_dist_, kf_angle_;
  int max_iter_;

  void cbQuat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    last_q_attitude_ = Eigen::Quaterniond(msg->quaternion.w,
                                         msg->quaternion.x,
                                         msg->quaternion.y,
                                         msg->quaternion.z);
  }

  // 🔥 IMU 기반 초기 추정
  Eigen::Isometry3d predictMotion(const rclcpp::Time& current_time)
  {
    if (first_) return Eigen::Isometry3d::Identity();
    
    static Eigen::Quaterniond q_prev = Eigen::Quaterniond::Identity();
    
    Eigen::Quaterniond q_curr = last_q_attitude_;
    Eigen::Quaterniond dq = q_curr * q_prev.inverse();
    q_prev = q_curr;
    
    // 큰 회전 변화 제한
    double angle = Eigen::AngleAxisd(dq).angle();
    if (std::abs(angle) > 0.5) {  // ~30도 이상 변화시 무시
      dq = Eigen::Quaterniond::Identity();
    }
    
    Eigen::Isometry3d prediction = Eigen::Isometry3d::Identity();
    prediction.rotate(dq);
    
    return prediction;
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
  {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc, *cloud);
    
    if (cloud->empty()) return;
    
    // 다운샘플링
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vg.setInputCloud(cloud);
    vg.filter(*cloud_filtered);

    if (first_) {
      keyframes_.push_back(cloud_filtered);
      *submap_ = *cloud_filtered;
      last_scan_time_ = pc->header.stamp;
      publishOdom(pc->header.stamp);
      first_ = false;
      RCLCPP_INFO(this->get_logger(), "First scan processed: %zu points", cloud_filtered->size());
      return;
    }

    // 🔥 IMU 기반 초기 추정
    Eigen::Isometry3d motion_pred = predictMotion(pc->header.stamp);
    Eigen::Isometry3d init_guess = T_world_curr_ * motion_pred;

    // GICP 수행
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> gicp;
    gicp.setMaxCorrespondenceDistance(max_corr_dist_);
    gicp.setMaximumIterations(max_iter_);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setInputSource(cloud_filtered);
    gicp.setInputTarget(submap_);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    gicp.align(aligned, init_guess.matrix().cast<float>());

    auto end_time = std::chrono::high_resolution_clock::now();
    double duration = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    if (!gicp.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "GICP failed to converge (%.1fms)", duration);
      return;
    }

    double fitness = gicp.getFitnessScore();
    if (fitness > 0.3) {
      RCLCPP_WARN(this->get_logger(), "Poor fitness: %.3f (%.1fms)", fitness, duration);
      return;
    }

    // 🔥 포즈 업데이트: 절대 좌표계 사용
    Eigen::Matrix4f T_result = gicp.getFinalTransformation();
    T_world_curr_ = Eigen::Isometry3d(T_result.cast<double>());

    // 🔥 키프레임 관리
    Eigen::Isometry3d delta_from_kf = last_kf_pose_.inverse() * T_world_curr_;
    bool is_keyframe = (delta_from_kf.translation().norm() > kf_dist_) ||
                       (Eigen::AngleAxisd(delta_from_kf.rotation()).angle() > kf_angle_);

    if (is_keyframe) {
      keyframes_.push_back(cloud_filtered);
      last_kf_pose_ = T_world_curr_;
      
      // 🔥 슬라이딩 윈도우 (최대 20개 키프레임)
      if (keyframes_.size() > 20) {
        keyframes_.pop_front();
      }
      
      // 서브맵 재구성
      submap_->clear();
      for (const auto& kf : keyframes_) {
        *submap_ += *kf;
      }
      
      // 서브맵 다운샘플링
      pcl::VoxelGrid<pcl::PointXYZI> vg_submap;
      vg_submap.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
      vg_submap.setInputCloud(submap_);
      vg_submap.filter(*submap_);
      
      RCLCPP_INFO(this->get_logger(), "New keyframe: %zu frames, submap: %zu points", 
                  keyframes_.size(), submap_->size());
    }

    publishOdom(pc->header.stamp);
    last_scan_time_ = pc->header.stamp;
    
    RCLCPP_DEBUG(this->get_logger(), "ICP: fitness=%.3f, time=%.1fms, points=%zu", 
                 fitness, duration, cloud_filtered->size());
  }

  void publishOdom(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    odom.pose.pose.position.x = T_world_curr_.translation().x();
    odom.pose.pose.position.y = T_world_curr_.translation().y();
    odom.pose.pose.position.z = T_world_curr_.translation().z();
    
    Eigen::Quaterniond q(T_world_curr_.rotation());
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    
    pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_q_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpOdom>());
  rclcpp::shutdown();
  return 0;
}
