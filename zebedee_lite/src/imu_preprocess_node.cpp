#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  const double alpha_ = 0.05;  // 저역통과 필터 계수
  double roll_filt_ = 0.0;
  double pitch_filt_ = 0.0;
  bool first_ = true;

  void cbImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // IMU 쿼터니언에서 RPY 추출
    tf2::Quaternion q_imu;
    tf2::fromMsg(msg->orientation, q_imu);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_imu).getRPY(roll, pitch, yaw);
    
    // 저역통과 필터 적용
    if(first_) {
      roll_filt_ = roll;
      pitch_filt_ = pitch;
      first_ = false;
    } else {
      roll_filt_ = alpha_ * roll + (1.0 - alpha_) * roll_filt_;
      pitch_filt_ = alpha_ * pitch + (1.0 - alpha_) * pitch_filt_;
    }
    
    // yaw는 0으로 고정, 필터링된 roll/pitch 사용
    tf2::Quaternion q_fixed;
    q_fixed.setRPY(roll_filt_, pitch_filt_, 0.0);
    q_fixed.normalize();
    
    geometry_msgs::msg::QuaternionStamped q;
    q.header = msg->header;
    q.quaternion = tf2::toMsg(q_fixed);
    pub_->publish(q);
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
