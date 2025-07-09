#include <iostream>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class ImuOdometryNode : public rclcpp::Node
{
public:
    ImuOdometryNode()
        : Node("imu_odometry_node"), x_(0.0), y_(0.0), vx_(0.0), vy_(0.0)
    {
        // QoS 설정을 EKF 노드의 기본값과 일치시킴 (Reliable)
        auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
        qos.reliable();

        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos, std::bind(&ImuOdometryNode::imu_callback, this, std::placeholders::_1));
            
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/imu_odom", qos);

        last_time_ = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "IMU Odometry Node started. (QoS Fixed & Sanity Check added)");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        rclcpp::Time current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt <= 0.0 || dt > 0.1) {
            return;
        }

        tf2::Quaternion orientation_q;
        tf2::fromMsg(msg->orientation, orientation_q);
        tf2::Matrix3x3 rotation_matrix(orientation_q);

        // 중력 벡터 계산
        tf2::Vector3 gravity_vector(0, 0, 9.80665);
        tf2::Vector3 gravity_in_sensor_frame = rotation_matrix.inverse() * gravity_vector;

        // 측정된 가속도에서 중력 제거
        double linear_accel_x = msg->linear_acceleration.x - gravity_in_sensor_frame.x();
        double linear_accel_y = msg->linear_acceleration.y - gravity_in_sensor_frame.y();

        // 안전 장치: 계산된 순수 가속도가 너무 크면 (예: 5 m/s^2 초과) 노이즈로 간주하고 무시
        if (std::sqrt(linear_accel_x * linear_accel_x + linear_accel_y * linear_accel_y) > 5.0) {
            // 속도를 0으로 리셋하여 드리프트 방지
            vx_ = 0.0;
            vy_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "High acceleration detected, ignoring measurement and resetting velocity.");
            return;
        }

        double roll, pitch, yaw;
        rotation_matrix.getRPY(roll, pitch, yaw);

        // 가속도를 월드 좌표계로 변환 (2D)
        double world_acc_x = linear_accel_x * cos(yaw) - linear_accel_y * sin(yaw);
        double world_acc_y = linear_accel_x * sin(yaw) + linear_accel_y * cos(yaw);

        // 적분
        vx_ += world_acc_x * dt;
        vy_ += world_acc_y * dt;
        x_ += vx_ * dt;
        y_ += vy_ * dt;
        
        // Odometry 메시지 발행
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.orientation = msg->orientation;
        odom_msg.twist.twist.linear.x = vx_;
        odom_msg.twist.twist.linear.y = vy_;
        odom_msg.twist.twist.angular.z = msg->angular_velocity.z;

        odom_publisher_->publish(odom_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    
    rclcpp::Time last_time_;
    double x_, y_, vx_, vy_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuOdometryNode>());
    rclcpp::shutdown();
    return 0;
}