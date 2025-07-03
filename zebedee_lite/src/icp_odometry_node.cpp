#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <deque>

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
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> kfs_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr submap_{new pcl::PointCloud<pcl::PointXYZI>()};
  bool first_=true;
  Eigen::Isometry3d T_w_last_ = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond last_q_attitude_ = Eigen::Quaterniond::Identity();
  
  const double KF_DIST = 0.30;   // 30cm
  const double KF_ANG  = 0.10;   // ~5.7도

  void cbQuat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    last_q_attitude_ = Eigen::Quaterniond(msg->quaternion.w,
                                         msg->quaternion.x,
                                         msg->quaternion.y,
                                         msg->quaternion.z);
  }

  Eigen::Isometry3d deltaPoseFromImu(const rclcpp::Time& t)
  {
    static rclcpp::Time last_t = t;
    static Eigen::Quaterniond q_last = Eigen::Quaterniond::Identity();
    
    Eigen::Quaterniond q = last_q_attitude_;
    Eigen::Quaterniond dq = q * q_last.inverse();
    q_last = q;
    
    double yaw = Eigen::AngleAxisd(dq).angle();
    if (std::abs(yaw) > M_PI) yaw = 0.0;  // 큰 점프 방지
    
    Eigen::Isometry3d inc = Eigen::Isometry3d::Identity();
    inc.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    return inc;
  }

  Eigen::Matrix4f eigenToMatrix(const Eigen::Isometry3d& T) {
    return T.matrix().cast<float>();
  }

  void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc, *cloud);

    if(first_){
      kfs_.push_back(cloud);
      *submap_ = *cloud;
      publishOdom(pc->header.stamp);
      first_=false;
      return;
    }

    // IMU 초기 guess
    Eigen::Isometry3d imu_inc = deltaPoseFromImu(pc->header.stamp);
    Eigen::Isometry3d init = T_w_last_ * imu_inc;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> gicp;
    gicp.setMaxCorrespondenceDistance(0.15);
    gicp.setMaximumIterations(80);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setInputSource(cloud);
    gicp.setInputTarget(submap_);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    gicp.align(aligned, eigenToMatrix(init));

    if(!gicp.hasConverged()) {
      RCLCPP_WARN(get_logger(), "GICP did not converge");
      return;
    }

    Eigen::Matrix4f T = gicp.getFinalTransformation();
    Eigen::Isometry3d T_inc(T.cast<double>());
    T_w_last_ = T_inc;

    // 키프레임 관리
    static Eigen::Isometry3d last_kf_pose = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d delta_from_kf = last_kf_pose.inverse() * T_w_last_;
    
    bool new_kf = (delta_from_kf.translation().norm() > KF_DIST) ||
                  (Eigen::AngleAxisd(delta_from_kf.rotation()).angle() > KF_ANG);

    if (new_kf) {
      kfs_.push_back(cloud);
      last_kf_pose = T_w_last_;
      
      if(kfs_.size() > 20) kfs_.pop_front();
      
      // 서브맵 재구성
      submap_->clear();
      for(auto& k : kfs_) *submap_ += *k;
      
      pcl::VoxelGrid<pcl::PointXYZI> vg; 
      vg.setLeafSize(0.05f,0.05f,0.05f);
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
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<IcpOdom>());
  rclcpp::shutdown();
  return 0;
}
