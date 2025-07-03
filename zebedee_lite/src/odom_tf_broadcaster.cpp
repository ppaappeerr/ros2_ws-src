#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class OdomTF : public rclcpp::Node
{
public:
  OdomTF() : Node("odom_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar_odom", 50,
      std::bind(&OdomTF::cbOdom, this, std::placeholders::_1));
  }

private:
  void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg->header;            // stamp+frame_id=odom
    tf.child_frame_id = msg->child_frame_id; // base_link
    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;
    tf.transform.rotation      = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<OdomTF>());
  rclcpp::shutdown();
  return 0;
}
