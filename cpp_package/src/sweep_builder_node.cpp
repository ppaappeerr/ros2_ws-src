#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <deque>
#include <mutex>

class SweepBuilderNode : public rclcpp::Node
{
public:
    SweepBuilderNode() : Node("sweep_builder_node")
    {
        // 파라미터 선언 및 로드
        this->declare_parameter<double>("sweep_duration", 1.0); // 1초를 하나의 스윕 주기로 설정
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("cloud_topic", "/sweep_cloud");
        this->declare_parameter<double>("lidar_z_offset", 0.071);

        sweep_duration_ = this->get_parameter("sweep_duration").as_double();
        lidar_z_offset_ = this->get_parameter("lidar_z_offset").as_double();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(50));

        // LiDAR 스캔 구독자
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->get_parameter("scan_topic").as_string(), qos,
            std::bind(&SweepBuilderNode::scanCallback, this, std::placeholders::_1));

        // IMU 데이터 버퍼링을 위한 구독자
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic").as_string(), qos,
            std::bind(&SweepBuilderNode::imuCallback, this, std::placeholders::_1));

        // 스윕 포인트 클라우드 발행자
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("cloud_topic").as_string(), 10);
        
        // 스윕 주기에 맞춰 클라우드를 발행할 타이머
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(sweep_duration_ * 1000)),
            std::bind(&SweepBuilderNode::publishSweep, this));
            
        RCLCPP_INFO(this->get_logger(), "Sweep Builder Node started. Sweeping for %.1f seconds.", sweep_duration_);
    }

private:
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;

    // IMU 데이터를 버퍼에 저장
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        imu_buffer_.push_back(*msg);
        // 너무 오래된 데이터는 삭제
        if (imu_buffer_.size() > 400) {
            imu_buffer_.pop_front();
        }
    }

    // LiDAR 스캔이 들어올 때마다 처리
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
        scan_buffer_.push_back(*msg);
    }
    
    // 타이머 주기마다 누적된 스캔으로 3D 클라우드 생성 및 발행
    void publishSweep()
    {
        std::deque<sensor_msgs::msg::LaserScan> current_scans;
        {
            std::lock_guard<std::mutex> lock(scan_buffer_mutex_);
            if (scan_buffer_.empty()) {
                return;
            }
            current_scans.swap(scan_buffer_); // 버퍼에 쌓인 스캔들을 가져옴
        }

        PointCloud::Ptr sweep_cloud(new PointCloud);
        
        // 가져온 모든 스캔에 대해 반복
        for (const auto& scan : current_scans)
        {
            // 각 스캔의 각 포인트에 대해 왜곡 보정 및 3D 변환 수행
            for (size_t i = 0; i < scan.ranges.size(); ++i)
            {
                float range = scan.ranges[i];
                if (std::isinf(range) || std::isnan(range) || range < scan.range_min || range > scan.range_max) {
                    continue;
                }

                // 1. 포인트의 정확한 측정 시간 계산
                rclcpp::Time point_time = rclcpp::Time(scan.header.stamp) + rclcpp::Duration::from_seconds(i * scan.time_increment);
                
                // 2. 해당 시간의 IMU 자세 보간
                tf2::Quaternion imu_orientation;
                if (!interpolateImu(point_time, imu_orientation)) {
                    continue; // 보간할 IMU 데이터가 없으면 스킵
                }
                
                // 3. IMU 좌표계 보정 (Roll/Pitch 반전 문제 해결)
                tf2::Quaternion correction;
                correction.setRPY(M_PI, 0, 0); // X축 기준 180도 회전
                tf2::Quaternion corrected_orientation = imu_orientation * correction;

                // 4. 2D 포인트를 3D로 변환
                float angle = scan.angle_min + i * scan.angle_increment;
                tf2::Vector3 point_2d(range * cos(angle), range * sin(angle), 0.0);

                // 5. 보정된 IMU 자세로 회전
                tf2::Vector3 point_3d = tf2::quatRotate(corrected_orientation, point_2d);

                // 6. Z축 옵셋 적용
                point_3d.setZ(point_3d.z() + lidar_z_offset_);

                sweep_cloud->points.emplace_back(point_3d.x(), point_3d.y(), point_3d.z());
            }
        }
        
        if (sweep_cloud->points.empty()) {
            return;
        }

        // 최종적으로 생성된 스윕 클라우드를 PointCloud2 메시지로 변환하여 발행
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*sweep_cloud, cloud_msg);
        cloud_msg.header.stamp = this->now();
        cloud_msg.header.frame_id = "base_link"; // 이 클라우드는 센서 기준
        
        cloud_pub_->publish(cloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published sweep cloud with %zu points.", sweep_cloud->points.size());
    }

    // 특정 시간의 IMU 자세를 선형 보간하는 함수
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
        double ratio = (time.seconds() - rclcpp::Time(before.header.stamp).seconds()) / (rclcpp::Time(after.header.stamp).seconds() - rclcpp::Time(before.header.stamp).seconds());
        
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
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<sensor_msgs::msg::LaserScan> scan_buffer_;
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    std::mutex scan_buffer_mutex_;
    std::mutex imu_buffer_mutex_;
    
    double sweep_duration_;
    double lidar_z_offset_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SweepBuilderNode>());
    rclcpp::shutdown();
    return 0;
}