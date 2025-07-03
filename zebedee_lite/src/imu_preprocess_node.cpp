#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Core>

class ImuPreprocess : public rclcpp::Node
{
public:
  ImuPreprocess() : Node("imu_preprocess")
  {
    sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 50,
        std::bind(&ImuPreprocess::cbImu, this, std::placeholders::_1));
    pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "/imu/attitude", 50);
  }

private:
  Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();
  
  void cbImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // LPF 0.02
    static Eigen::Vector3d gyro_f = {0,0,0};
    Eigen::Vector3d g_raw(msg->angular_velocity.x,
                          msg->angular_velocity.y,
                          msg->angular_velocity.z);
    gyro_f = 0.02 * g_raw + 0.98 * gyro_f;

    tf2::Quaternion q_in;
    tf2::fromMsg(msg->orientation, q_in);
    double r,p,y;
    tf2::Matrix3x3(q_in).getRPY(r,p,y);
    y = 0.0;                                // yaw 제거
    tf2::Quaternion q_out;
    q_out.setRPY(r,p,y); q_out.normalize();

    geometry_msgs::msg::QuaternionStamped out;
    out.header = msg->header;
    out.quaternion = tf2::toMsg(q_out);
    pub_->publish(out);

    last_q_ = Eigen::Quaterniond(q_out.w(),q_out.x(),q_out.y(),q_out.z());
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPreprocess>());
  rclcpp::shutdown();
  return 0;
}
