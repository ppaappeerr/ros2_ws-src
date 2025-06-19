#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// 1단계에서 추가한 PCL 관련 헤더
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 2단계에서 새로 만든 커스텀 포인트 타입 헤더 포함
#include "cpp_package/point_types.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class PointCloudFormatterNode : public rclcpp::Node
{
public:
    PointCloudFormatterNode() : Node("pointcloud_formatter_node")
    {
        auto sensor_qos = rclcpp::SensorDataQoS();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sensor_qos, std::bind(&PointCloudFormatterNode::scan_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sensor_qos, std::bind(&PointCloudFormatterNode::imu_callback, this, std::placeholders::_1));

        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/formatted_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "C++ PointCloud Formatter Node has been started.");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        // 1. IMU 데이터가 없으면 아무 작업도 하지 않음
        if (!latest_imu_msg_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "No IMU data received yet. Skipping scan.");
            return;
        }

        // 2. 커스텀 포인트 타입을 사용하는 PCL 포인트 클라우드 생성
        pcl::PointCloud<PointLIO> cloud_out;
        cloud_out.header.frame_id = "laser"; // PCL 헤더에 프레임 ID 설정
        pcl_conversions::toPCL(scan_msg->header.stamp, cloud_out.header.stamp);

        // IMU 데이터로부터 회전(Quaternion) 정보 가져오기
        tf2::Quaternion imu_quat;
        tf2::fromMsg(latest_imu_msg_->orientation, imu_quat);

        // 3. LaserScan의 각 포인트를 순회하며 변환
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float range = scan_msg->ranges[i];

            // 유효한 거리 값만 처리
            if (std::isinf(range) || std::isnan(range)) {
                continue;
            }

            // 2D 좌표 계산
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

            // 3D로 변환 (아직은 Z=0 평면에 있음)
            tf2::Vector3 point_2d(range * cos(angle), range * sin(angle), 0.0);

            // IMU 회전을 적용하여 3D 좌표로 변환
            tf2::Vector3 point_3d = tf2::quatRotate(imu_quat, point_2d);

            // 4. 우리만의 커스텀 포인트(PointLIO)를 만들고 값 채우기
            PointLIO point;
            point.x = point_3d.x();
            point.y = point_3d.y();
            point.z = point_3d.z();

            // 누락된 필드들을 임의의 값으로 채워넣기
            point.intensity = 1.0f; // 강도 정보가 없으므로 1.0으로 고정
            point.tag = 0;          // 태그 정보 없으므로 0으로 고정
            point.line = 0;         // 2D LiDAR는 라인이 1개이므로 0으로 고정

            // TODO (3단계): 여기에 정확한 타임스탬프 계산 로직 추가
            point.timestamp = 0.0; // 우선 0.0으로 초기화

            cloud_out.points.push_back(point);
        }

        // 5. PCL 포인트 클라우드를 ROS2 메시지로 변환하여 발행
        sensor_msgs::msg::PointCloud2 cloud_msg_out;
        pcl::toROSMsg(cloud_out, cloud_msg_out);
        cloud_msg_out.header.stamp = scan_msg->header.stamp; // 원본 스캔의 타임스탬프 사용
        cloud_msg_out.header.frame_id = "laser";          // 프레임 ID 설정
        cloud_pub_->publish(cloud_msg_out);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_msg_ = msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_msg_ = nullptr;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudFormatterNode>());
    rclcpp::shutdown();
    return 0;
}