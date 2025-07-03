#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class DeskewScan : public rclcpp::Node
{
public:
  DeskewScan() : Node("deskew_scan")
  {
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&DeskewScan::cbScan, this, std::placeholders::_1));
    sub_q_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
        "/imu/attitude", 50,
        std::bind(&DeskewScan::cbQuat, this, std::placeholders::_1));
    pub_pc_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/deskewed_cloud", 10);
  }

private:
  struct QStamped{double t; Eigen::Quaterniond q;};
  std::deque<QStamped> qbuf_;

  void cbQuat(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
  {
    QStamped elem;
    elem.t = rclcpp::Time(msg->header.stamp).seconds();
    elem.q = Eigen::Quaterniond(msg->quaternion.w,
                                msg->quaternion.x,
                                msg->quaternion.y,
                                msg->quaternion.z);
    qbuf_.push_back(elem);
    while(qbuf_.size()>200) qbuf_.pop_front();
  }

  Eigen::Quaterniond interp(double t)
  {
    if(qbuf_.size()<2) return Eigen::Quaterniond::Identity();
    for(size_t i=1;i<qbuf_.size();++i){
      if(t < qbuf_[i].t){
        double ratio = (t - qbuf_[i-1].t)/(qbuf_[i].t - qbuf_[i-1].t);
        return qbuf_[i-1].q.slerp(ratio, qbuf_[i].q);
      }
    }
    return qbuf_.back().q;
  }

  void cbScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
                   new pcl::PointCloud<pcl::PointXYZI>);
    double t0 = rclcpp::Time(scan->header.stamp).seconds();

    for(size_t i=0;i<scan->ranges.size();++i){
      float r = scan->ranges[i];
      if(std::isnan(r) || r<0.05 || r>scan->range_max) continue;
      double ang = scan->angle_min + i*scan->angle_increment;
      double t_pt = t0 + i*scan->time_increment;

      Eigen::Quaterniond q = interp(t_pt);
      Eigen::Vector3d p_local(r*std::cos(ang), r*std::sin(ang), 0.0);
      Eigen::Vector3d p_world = q * p_local;

      pcl::PointXYZI pt;
      pt.x = p_world.x(); pt.y = p_world.y(); pt.z = p_world.z();
      pt.intensity = 1.0;
      cloud->push_back(pt);
    }

    // 가벼운 Voxel (옵션)
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(0.05f,0.05f,0.05f);
    vg.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZI> cloud_filt;
    vg.filter(cloud_filt);

    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(cloud_filt, out);
    out.header = scan->header;
    pub_pc_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_q_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pc_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<DeskewScan>());
  rclcpp::shutdown();
  return 0;
}
