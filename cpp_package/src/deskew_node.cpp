#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>
#include <mutex>

class DeskewNode : public rclcpp::Node
{
public:
    DeskewNode() : Node("deskew_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("output_cloud_topic", "/corrected_cloud");
        this->declare_parameter<double>("lidar_z_offset", 0.071);

        lidar_z_offset_ = this->get_parameter("lidar_z_offset").as_double();
        
        auto qos = rclcpp::QoS(rclcpp::KeepLast(50));

        // LiDAR 스캔이 들어오는 즉시 처리할 구독자
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), qos,
            std::bind(&DeskewNode::scanCallback, this, std::placeholders::_1));

        // IMU 데이터는 버퍼에만 저장
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), qos,
            std::bind(&DeskewNode::imuCallback, this, std::placeholders::_1));

        // 처리된 포인트 클라우드 발행자
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("output_cloud_topic").as_string(), 10);
            
        RCLCPP_INFO(this->get_logger(), "Real-time Deskew Node has started.");
    }

private:
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;

    // IMU 데이터를 버퍼에 저장
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        imu_buffer_.push_back(*msg);
        if (imu_buffer_.size() > 400) { // 너무 오래된 데이터는 삭제
            imu_buffer_.pop_front();
        }
    }

    // LiDAR 스캔이 들어올 때마다 실시간으로 처리 및 발행
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        PointCloud::Ptr corrected_cloud(new PointCloud());
        
        // 각 스캔의 모든 포인트에 대해 왜곡 보정 및 3D 변환 수행
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];
            if (std::isinf(range) || std::isnan(range) || range < scan->range_min || range > scan->range_max) {
                continue;
            }

            // 1. 포인트의 정확한 측정 시간 계산
            rclcpp::Time point_time = rclcpp::Time(scan->header.stamp) + rclcpp::Duration::from_seconds(i * scan->time_increment);
            
            // 2. 해당 시간의 IMU 자세 보간
            tf2::Quaternion imu_orientation;
            if (!interpolateImu(point_time, imu_orientation)) {
                continue; 
            }
            
            // 3. IMU 좌표계 보정
            tf2::Quaternion corrected_orientation = imu_orientation;

            // 4. 2D 포인트를 3D로 변환
            float angle = scan->angle_min + i * scan->angle_increment;
            tf2::Vector3 point_2d(range * cos(angle), range * sin(angle), 0.0);

            // 5. 보정된 IMU 자세로 회전
            tf2::Vector3 point_3d = tf2::quatRotate(corrected_orientation, point_2d);

            // 6. Z축 옵셋 적용
            point_3d.setZ(point_3d.z() + lidar_z_offset_);

            corrected_cloud->points.emplace_back(point_3d.x(), point_3d.y(), point_3d.z());
        }
        
        if (corrected_cloud->points.empty()) {
            return;
        }

        // 최종 클라우드를 PointCloud2 메시지로 변환하여 즉시 발행
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*corrected_cloud, cloud_msg);
        cloud_msg.header = scan->header;
        cloud_msg.header.frame_id = "base_link"; 
        
        cloud_pub_->publish(cloud_msg);
    }

    // IMU 보간 함수 (이전과 동일)
    bool interpolateImu(const rclcpp::Time& time, tf2::Quaternion& output_orientation)
    {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        if (imu_buffer_.size() < 2) return false;

        auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), time,
            [](const sensor_msgs::msg::Imu& imu, const rclcpp::Time& t) {
                return rclcpp::Time(imu.header.stamp) < t;
            });

        if (it == imu_buffer_.begin() || it == imu_buffer_.end()) return false;

        const auto& after = *it;
        const auto& before = *(it - 1);
        double total_dt = (rclcpp::Time(after.header.stamp) - rclcpp::Time(before.header.stamp)).seconds();
        if (total_dt <= 0) return false;
        double ratio = (time.seconds() - rclcpp::Time(before.header.stamp).seconds()) / total_dt;
        
        tf2::Quaternion q_before, q_after;
        tf2::fromMsg(before.orientation, q_before);
        tf2::fromMsg(after.orientation, q_after);
        
        output_orientation = q_before.slerp(q_after, ratio).normalized();
        return true;
    }

    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    std::mutex imu_buffer_mutex_;
    
    double lidar_z_offset_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeskewNode>());
    rclcpp::shutdown();
    return 0;
}
