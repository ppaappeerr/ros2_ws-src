#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <deque>

class Icp3dOdomNode : public rclcpp::Node
{
public:
    Icp3dOdomNode() : Node("icp_3d_odom_node"), last_cloud_initialized_(false)
    {
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("voxel_leaf_size", 0.05);
        this->get_parameter("odom_frame", odom_frame_);
        this->get_parameter("base_frame", base_frame_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        scan_sub_.subscribe(this, "/scan", qos.get_rmw_qos_profile());
        imu_sub_.subscribe(this, "/imu/data", qos.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), scan_sub_, imu_sub_
        );
        sync_->registerCallback(std::bind(&Icp3dOdomNode::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

        raw_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos, std::bind(&Icp3dOdomNode::imuCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_icp", qos);
        cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/corrected_cloud", qos);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        icp_.setMaxCorrespondenceDistance(0.15);
        icp_.setTransformationEpsilon(1e-9);
        icp_.setEuclideanFitnessEpsilon(1e-9);
        icp_.setMaximumIterations(150);
        
        odom_to_base_transform_.setIdentity();
        last_scan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

        RCLCPP_INFO(this->get_logger(), "ICP 3D Odometry Node with Motion Correction and IMU Pre-integration has been started.");
    }

private:
    // ==========================================================================================
    // 메인 콜백 함수: IMU 예측값을 ICP 초기값으로 사용하여 Odometry 추정
    // ==========================================================================================
    void syncCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg, const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
    {
        if (!last_cloud_initialized_) {
            last_scan_time_ = scan_msg->header.stamp;
            last_cloud_ = createCorrectedCloud(scan_msg);
            last_cloud_initialized_ = true;
            return;
        }

        // 1. IMU Pre-integration으로 초기 변환 행렬 추정
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        integrateImu(scan_msg->header.stamp, initial_guess);

        // 2. 현재 스캔의 모션 왜곡 보정된 포인트 클라우드 생성
        auto corrected_cloud_raw = createCorrectedCloud(scan_msg);
        if(corrected_cloud_raw->points.empty()){
            RCLCPP_WARN(this->get_logger(), "Corrected cloud is empty, skipping frame.");
            return;
        }
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(corrected_cloud_raw);
        sor.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr corrected_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        sor.filter(*corrected_cloud);

        // 3. ICP 수행 (IMU 예측값을 초기값으로 사용)
        icp_.setInputSource(corrected_cloud);
        icp_.setInputTarget(last_cloud_);
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp_.align(aligned_cloud, initial_guess); // 두 번째 인자로 초기값 전달

        if (icp_.hasConverged()) {
            Eigen::Matrix4f incremental_transform_eigen = icp_.getFinalTransformation();
            tf2::Transform incremental_transform;
            tf2::fromEigen(incremental_transform_eigen.cast<double>(), incremental_transform);
            
            odom_to_base_transform_ = odom_to_base_transform_ * incremental_transform;

            publishOdometry(scan_msg->header.stamp, odom_to_base_transform_);
            publishTf(scan_msg->header.stamp, odom_to_base_transform_);

            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*corrected_cloud, cloud_msg);
            cloud_msg.header = scan_msg->header;
            cloud_msg.header.frame_id = "base_link";
            cloud_pub_->publish(cloud_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }

        last_cloud_ = corrected_cloud;
        last_scan_time_ = scan_msg->header.stamp;
    }

    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        imu_buffer_.push_back(*msg);
        if (imu_buffer_.size() > 400) { // 4초 분량 (100Hz 기준)
            imu_buffer_.pop_front();
        }
    }

    // ==========================================================================================
    // [NEW] IMU Pre-integration 함수: 이전 스캔과 현재 스캔 사이의 IMU 데이터를 적분하여 상대 변환 예측
    // ==========================================================================================
    void integrateImu(const rclcpp::Time& current_scan_time, Eigen::Matrix4f& initial_guess) {
        if (imu_buffer_.empty() || rclcpp::Time(imu_buffer_.front().header.stamp) > last_scan_time_) {
            return; // 적분할 데이터 없음
        }

        tf2::Quaternion orientation_change(0, 0, 0, 1);

        for (size_t i = 0; i < imu_buffer_.size() - 1; ++i) {
            const auto& imu_before = imu_buffer_[i];
            const auto& imu_after = imu_buffer_[i+1];
            rclcpp::Time time_before = imu_before.header.stamp;
            rclcpp::Time time_after = imu_after.header.stamp;

            if (time_before >= last_scan_time_ && time_after <= current_scan_time) {
                double dt = (time_after - time_before).seconds();
                tf2::Vector3 angular_velocity(
                    (imu_before.angular_velocity.x + imu_after.angular_velocity.x) / 2.0,
                    (imu_before.angular_velocity.y + imu_after.angular_velocity.y) / 2.0,
                    (imu_before.angular_velocity.z + imu_after.angular_velocity.z) / 2.0
                );
                
                tf2::Quaternion delta_q(angular_velocity.x() * dt, angular_velocity.y() * dt, angular_velocity.z() * dt, 0);
                delta_q = delta_q * 0.5 * orientation_change;
                orientation_change = orientation_change + delta_q;
                orientation_change.normalize();
            }
        }

        Eigen::Matrix3d rotation_matrix = tf2::transformToEigen(tf2::Transform(orientation_change)).rotation();
        initial_guess.block<3,3>(0,0) = rotation_matrix.cast<float>();
    }

    // 모션 왜곡 보정 함수 (이전과 동일)
    pcl::PointCloud<pcl::PointXYZ>::Ptr createCorrectedCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        rclcpp::Time scan_start_time = scan_msg->header.stamp;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            if (std::isinf(range) || std::isnan(range) || range < scan_msg->range_min || range > scan_msg->range_max) {
                continue;
            }

            rclcpp::Time point_time = scan_start_time + rclcpp::Duration::from_seconds(i * scan_msg->time_increment);
            tf2::Quaternion orientation;
            if (!interpolateImu(point_time, orientation)) {
                continue; 
            }

            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            tf2::Vector3 point_lidar_frame(range * cos(angle), range * sin(angle), 0.0);
            tf2::Vector3 point_base_frame = tf2::quatRotate(orientation, point_lidar_frame);

            cloud->points.emplace_back(point_base_frame.x(), point_base_frame.y(), point_base_frame.z());
        }
        return cloud;
    }

    // IMU 보간 함수 (이전과 동일)
    bool interpolateImu(const rclcpp::Time& time, tf2::Quaternion& output_orientation) {
        if (imu_buffer_.size() < 2) return false;
        
        auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), time, 
            [](const sensor_msgs::msg::Imu& imu_msg, const rclcpp::Time& t) {
                return rclcpp::Time(imu_msg.header.stamp) < t;
            });
        
        if (it == imu_buffer_.begin() || it == imu_buffer_.end()) return false;

        const auto& after = *it;
        const auto& before = *(it - 1);

        rclcpp::Time before_time = before.header.stamp;
        rclcpp::Time after_time = after.header.stamp;
        double ratio = (time.seconds() - before_time.seconds()) / (after_time.seconds() - before_time.seconds());

        tf2::Quaternion q_before, q_after;
        tf2::fromMsg(before.orientation, q_before);
        tf2::fromMsg(after.orientation, q_after);
        
        output_orientation = q_before.slerp(q_after, ratio).normalized();
        return true;
    }

    // Odometry 및 TF 발행 함수 (이전과 동일)
    void publishOdometry(const rclcpp::Time& stamp, const tf2::Transform& transform) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose.pose = tf2::toMsg(transform);
        odom_pub_->publish(odom_msg);
    }
    void publishTf(const rclcpp::Time& stamp, const tf2::Transform& transform) {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = stamp;
        tf_stamped.header.frame_id = odom_frame_;
        tf_stamped.child_frame_id = base_frame_;
        tf_stamped.transform = tf2::toMsg(transform);
        tf_broadcaster_->sendTransform(tf_stamped);
    }

    // 멤버 변수
    std::string odom_frame_, base_frame_;
    double voxel_leaf_size_;
    rclcpp::Time last_scan_time_;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::Imu>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_cloud_;
    bool last_cloud_initialized_;

    tf2::Transform odom_to_base_transform_;
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Icp3dOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}