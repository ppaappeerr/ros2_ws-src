#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>

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
  }

private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr submap_{new pcl::PointCloud<pcl::PointXYZI>};
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> keyframes_;
  bool first_=true;
  Eigen::Isometry3d T_w_last_ = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond last_q_attitude_ = Eigen::Quaterniond::Identity();
  rclcpp::Time last_imu_time_;
  
  const double KF_DIST = 0.30;   // 30cm
  const double KF_ANG  = 0.10;   // ~5.7도

  void cbQuat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    last_q_attitude_ = Eigen::Quaterniond(msg->quaternion.w,
                                         msg->quaternion.x,
                                         msg->quaternion.y,
                                         msg->quaternion.z);
    last_imu_time_ = msg->header.stamp;
  }

  Eigen::Isometry3d deltaPoseFromImu(const rclcpp::Time& stamp)
  {
    static rclcpp::Time last_t = stamp;
    static Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
    
    double dt = (stamp - last_t).seconds();
    last_t = stamp;
    
    if(dt > 0.1) { // 너무 오래된 데이터면 무시
      q0 = last_q_attitude_;
      return Eigen::Isometry3d::Identity();
    }
    
    Eigen::Quaterniond q = last_q_attitude_;
    Eigen::Quaterniond delta_q = q * q0.inverse();
    q0 = q;
    
    Eigen::Isometry3d inc = Eigen::Isometry3d::Identity();
    inc.rotate(delta_q);
    return inc;
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc, *cloud);

    if(first_){
      keyframes_.push_back(cloud);
      *submap_ = *cloud;
      publishOdom(pc->header.stamp);
      first_=false;
      return;
    }

    pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(0.5);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setInputSource(cloud);
    icp.setInputTarget(submap_);

    // IMU 초기 guess 추가
    Eigen::Isometry3d imu_delta = deltaPoseFromImu(pc->header.stamp);
    Eigen::Isometry3d init = T_w_last_ * imu_delta;
    Eigen::Matrix4f init_guess = init.matrix().cast<float>();
    
    pcl::PointCloud<pcl::PointXYZI> aligned;
    icp.align(aligned, init_guess);

    if(!icp.hasConverged()) {
      RCLCPP_WARN(get_logger(), "ICP did not converge");
      return;
    }

    Eigen::Matrix4f T = icp.getFinalTransformation();
    Eigen::Isometry3d T_inc(T.cast<double>());
    T_w_last_ = T_inc;

    // ✅ 수정: 키프레임 관리 로직 개선
    Eigen::Isometry3d delta_from_last_kf = last_keyframe_pose_.inverse() * T_w_last_;
    
    bool need_new_kf = (delta_from_last_kf.translation().norm() > KF_DIST) ||
                       (Eigen::AngleAxisd(delta_from_last_kf.rotation()).angle() > KF_ANG);

    if(need_new_kf) {
      keyframes_.push_back(cloud);
      last_keyframe_pose_ = T_w_last_;  // 마지막 키프레임 위치 업데이트
      
      if(keyframes_.size() > 20) {
        keyframes_.pop_front();
      }
      
      // 서브맵 재구성
      submap_->clear();
      for(auto& kf : keyframes_) {
        *submap_ += *kf;
      }
      
      // 다운샘플링
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setLeafSize(0.05f, 0.05f, 0.05f);
      vg.setInputCloud(submap_);
      vg.filter(*submap_);
    }

    publishOdom(pc->header.stamp);
  }

  void publishOdom(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = T_w_last_.translation().x();
    odom.pose.pose.position.y = T_w_last_.translation().y();
    odom.pose.pose.position.z = T_w_last_.translation().z();
    Eigen::Quaterniond q(T_w_last_.rotation());
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_q_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  Eigen::Isometry3d last_keyframe_pose_ = Eigen::Isometry3d::Identity();
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<IcpOdom>());
  rclcpp::shutdown();
  return 0;
}
