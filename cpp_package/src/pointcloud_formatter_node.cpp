#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "cpp_package/point_types.h" // 2단계에서 만든 커스텀 포인트 타입

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <algorithm>

class PointCloudFormatterNode : public rclcpp::Node
{
public:
    PointCloudFormatterNode() : Node("pointcloud_formatter_node")
    {
        // QoS 설정: 센서 데이터는 Best-Effort, 발행은 RViz2와 호환되도록 Reliable
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort();
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", sub_qos, std::bind(&PointCloudFormatterNode::scan_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", sub_qos, std::bind(&PointCloudFormatterNode::imu_callback, this, std::placeholders::_1));
        
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/formatted_cloud", pub_qos);

        this->declare_parameter<double>("imu_buffer_seconds", 2.0);
        imu_buffer_duration_ = std::chrono::duration<double>(this->get_parameter("imu_buffer_seconds").as_double());

        RCLCPP_INFO(this->get_logger(), "PointCloud Formatter Final Version (IMU Slerp Interpolation) started.");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_buffer_.push_back(msg);
        rclcpp::Time now = this->get_clock()->now();
        while (!imu_buffer_.empty() && (now - imu_buffer_.front()->header.stamp) > imu_buffer_duration_) {
            imu_buffer_.pop_front();
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        if (imu_buffer_.size() < 2) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "IMU buffer is not ready for interpolation. (Need at least 2 messages)");
            return;
        }

        rclcpp::Time scan_start_time = scan_msg->header.stamp;
        double scan_duration = scan_msg->time_increment * (scan_msg->ranges.size() - 1);
        rclcpp::Time scan_end_time = scan_start_time + rclcpp::Duration::from_seconds(scan_duration);

        // [안전 장치] 스캔 전체를 커버할 만큼의 IMU 데이터가 버퍼에 있는지 확인
        if (scan_end_time > imu_buffer_.back()->header.stamp) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Scan data is newer than IMU buffer. Waiting for more IMU data.");
            return;
        }

        pcl::PointCloud<PointLIO> cloud_out;
        cloud_out.header.frame_id = "laser";
        pcl_conversions::toPCL(scan_start_time, cloud_out.header.stamp);

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            float range = scan_msg->ranges[i];
            if (range < scan_msg->range_min || range > scan_msg->range_max) continue;

            double point_time_offset_sec = scan_msg->time_increment * i;
            rclcpp::Time point_time = scan_start_time + rclcpp::Duration::from_seconds(point_time_offset_sec);

            Eigen::Quaterniond point_orientation;
            if (!interpolate_orientation(point_time, point_orientation)) continue;

            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // IMU 좌표계 기준 2D 포인트를 생성 후, 보간된 방향으로 회전시켜 3D 포인트 생성
            Eigen::Vector3d point_2d(range * cos(angle), range * sin(angle), 0.0);
            Eigen::Vector3d point_3d = point_orientation.conjugate() * point_2d;

            PointLIO point;
            point.x = static_cast<float>(point_3d.x());
            point.y = static_cast<float>(point_3d.y());
            point.z = static_cast<float>(point_3d.z());
            point.intensity = 1.0f;
            point.tag = 0;
            point.line = 0;
            point.timestamp = point_time_offset_sec; // Fast-LIO는 초 단위의 상대 시간을 사용

            cloud_out.points.push_back(point);
        }

        if (cloud_out.points.empty()) return;

        sensor_msgs::msg::PointCloud2 cloud_msg_out;
        pcl::toROSMsg(cloud_out, cloud_msg_out);
        cloud_msg_out.header.stamp = scan_start_time;
        cloud_msg_out.header.frame_id = "laser";
        cloud_pub_->publish(cloud_msg_out);
    }
    
    bool interpolate_orientation(const rclcpp::Time& target_time, Eigen::Quaterniond& output)
    {
        if (imu_buffer_.size() < 2) return false;

        auto it_after = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), target_time, 
            [](const sensor_msgs::msg::Imu::SharedPtr& msg, const rclcpp::Time& t) {
                return rclcpp::Time(msg->header.stamp) < t;
            });

        if (it_after == imu_buffer_.begin() || it_after == imu_buffer_.end()) return false;

        auto it_before = std::prev(it_after);
        
        const auto& imu1 = *it_before;
        const auto& imu2 = *it_after;
        rclcpp::Time t1 = imu1->header.stamp;
        rclcpp::Time t2 = imu2->header.stamp;

        double total_diff = (t2 - t1).seconds();
        if (total_diff <= 1e-6) {
             output = Eigen::Quaterniond(imu1->orientation.w, imu1->orientation.x, imu1->orientation.y, imu1->orientation.z);
             return true;
        }

        double factor = (target_time - t1).seconds() / total_diff;
        Eigen::Quaterniond q1(imu1->orientation.w, imu1->orientation.x, imu1->orientation.y, imu1->orientation.z);
        Eigen::Quaterniond q2(imu2->orientation.w, imu2->orientation.x, imu2->orientation.y, imu2->orientation.z);
        
        output = q1.slerp(factor, q2).normalized();
        return true;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    std::mutex imu_mutex_;
    std::chrono::duration<double> imu_buffer_duration_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudFormatterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}