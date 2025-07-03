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
#include <mutex>
#include <chrono>

class ICP3DOdomNode : public rclcpp::Node
{
public:
    ICP3DOdomNode() : Node("icp_3d_odom_cpp")
    {
        // 파라미터
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("scan_voxel_size", 0.05);
        this->declare_parameter<double>("submap_voxel_size", 0.1);
        this->declare_parameter<double>("max_correspondence_distance", 0.2);
        this->declare_parameter<int>("max_iterations", 50);
        this->declare_parameter<int>("submap_keyframes", 20);
        this->declare_parameter<double>("keyframe_threshold_dist", 0.5);
        this->declare_parameter<double>("keyframe_threshold_rot", 0.2); // radians
        
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        scan_voxel_size_ = this->get_parameter("scan_voxel_size").as_double();
        submap_voxel_size_ = this->get_parameter("submap_voxel_size").as_double();
        submap_keyframes_ = this->get_parameter("submap_keyframes").as_int();
        keyframe_threshold_dist_ = this->get_parameter("keyframe_threshold_dist").as_double();
        keyframe_threshold_rot_ = this->get_parameter("keyframe_threshold_rot").as_double();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        scan_sub_.subscribe(this, "/scan", qos.get_rmw_qos_profile());
        imu_sub_.subscribe(this, "/imu/data", qos.get_rmw_qos_profile());
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), scan_sub_, imu_sub_);
        sync_->registerCallback(std::bind(&ICP3DOdomNode::scanImuCallback, this, std::placeholders::_1, std::placeholders::_2));

        raw_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", qos, std::bind(&ICP3DOdomNode::imuBufferCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_icp", qos);
        submap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", qos);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        icp_.setMaxCorrespondenceDistance(this->get_parameter("max_correspondence_distance").as_double());
        icp_.setTransformationEpsilon(1e-9);
        icp_.setEuclideanFitnessEpsilon(1e-9);
        icp_.setMaximumIterations(this->get_parameter("max_iterations").as_int());
        
        current_pose_.setIdentity();
        last_keyframe_pose_.setIdentity();
        last_scan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

        RCLCPP_INFO(this->get_logger(), "Scan-to-Map Odometry with Keyframes started.");
    }

private:
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;
    using PointCloudPtr = PointCloud::Ptr;

    void scanImuCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg, 
                         const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
    {
        (void)imu_msg;  // 🔥 경고 제거
        
        PointCloudPtr current_scan = createCorrectedCloud(scan_msg);
        if(!current_scan || current_scan->points.empty()){
            return;
        }

        if (keyframe_deque_.empty()) {
            addKeyframe(current_scan, current_pose_);
            last_scan_time_ = scan_msg->header.stamp;
            RCLCPP_INFO(this->get_logger(), "First keyframe added.");
            return;
        }
        
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        integrateImuRotation(scan_msg->header.stamp, initial_guess);

        icp_.setInputSource(current_scan);
        icp_.setInputTarget(submap_cloud_);
        PointCloud aligned_cloud;
        icp_.align(aligned_cloud, initial_guess);

        if (icp_.hasConverged()) {
            current_pose_ = icp_.getFinalTransformation();
            
            publishOdometry(scan_msg->header.stamp, current_pose_);
            publishTf(scan_msg->header.stamp, current_pose_);

            if (isNewKeyframe(current_pose_)) {
                addKeyframe(current_scan, current_pose_);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge.");
        }
        last_scan_time_ = scan_msg->header.stamp;
    }

    void addKeyframe(const PointCloudPtr& scan, const Eigen::Matrix4f& pose) {
        PointCloudPtr scan_world(new PointCloud);
        pcl::transformPointCloud(*scan, *scan_world, pose);
        keyframe_deque_.push_back(scan_world);

        if (keyframe_deque_.size() > (size_t)submap_keyframes_) {
            keyframe_deque_.pop_front();
        }
        
        // 슬라이딩 윈도우 방식으로 서브맵 업데이트
        PointCloudPtr new_submap(new PointCloud);
        for(const auto& frame : keyframe_deque_){
            *new_submap += *frame;
        }
        
        // 서브맵 다운샘플링
        pcl::VoxelGrid<PointType> voxel_filter;
        voxel_filter.setInputCloud(new_submap);
        voxel_filter.setLeafSize(submap_voxel_size_, submap_voxel_size_, submap_voxel_size_);
        submap_cloud_.reset(new PointCloud());
        voxel_filter.filter(*submap_cloud_);
        
        last_keyframe_pose_ = pose;

        // 디버깅용 서브맵 발행
        sensor_msgs::msg::PointCloud2 submap_msg;
        pcl::toROSMsg(*submap_cloud_, submap_msg);
        submap_msg.header.stamp = this->now();
        submap_msg.header.frame_id = odom_frame_;
        submap_pub_->publish(submap_msg);
    }
    
    bool isNewKeyframe(const Eigen::Matrix4f& pose) {
        Eigen::Vector3f trans_diff = pose.block<3,1>(0,3) - last_keyframe_pose_.block<3,1>(0,3);
        Eigen::Matrix3f rot_diff_mat = last_keyframe_pose_.block<3,3>(0,0).transpose() * pose.block<3,3>(0,0);
        Eigen::AngleAxisf rot_diff_angle_axis(rot_diff_mat);

        return trans_diff.norm() > keyframe_threshold_dist_ || std::abs(rot_diff_angle_axis.angle()) > keyframe_threshold_rot_;
    }

    void imuBufferCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        imu_buffer_.push_back(*msg);
        if (imu_buffer_.size() > 400) imu_buffer_.pop_front();
    }

    // 🔥 수정: IMU Pre-integration - tf2 변환 함수 사용 안함
    void integrateImuRotation(const rclcpp::Time& current_scan_time, Eigen::Matrix4f& initial_guess) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        if (imu_buffer_.empty() || rclcpp::Time(imu_buffer_.front().header.stamp) > last_scan_time_) return;
        
        tf2::Quaternion orientation_change(0, 0, 0, 1);
        
        for (size_t i = 0; i < imu_buffer_.size() - 1; ++i) {
            const auto& imu0 = imu_buffer_[i]; 
            const auto& imu1 = imu_buffer_[i+1];
            rclcpp::Time time0 = imu0.header.stamp; 
            rclcpp::Time time1 = imu1.header.stamp;
            
            if (time0 >= last_scan_time_ && time1 <= current_scan_time) {
                double dt = (time1 - time0).seconds();
                if (dt > 0.0 && dt < 0.1) {  // 유효한 시간 간격만 사용
                    tf2::Vector3 ang_vel(
                        (imu0.angular_velocity.x + imu1.angular_velocity.x) / 2.0,
                        (imu0.angular_velocity.y + imu1.angular_velocity.y) / 2.0,
                        (imu0.angular_velocity.z + imu1.angular_velocity.z) / 2.0
                    );
                    tf2::Quaternion delta_q(ang_vel.x() * dt, ang_vel.y() * dt, ang_vel.z() * dt, 0);
                    delta_q = delta_q * 0.5 * orientation_change;
                    orientation_change = (orientation_change + delta_q).normalized();
                }
            }
        }
        
        // 🔥 수정: tf2::Quaternion을 직접 Eigen::Matrix3f로 변환
        tf2::Matrix3x3 rotation_matrix;
        rotation_matrix.setRotation(orientation_change);
        
        initial_guess(0,0) = rotation_matrix[0][0]; initial_guess(0,1) = rotation_matrix[0][1]; initial_guess(0,2) = rotation_matrix[0][2];
        initial_guess(1,0) = rotation_matrix[1][0]; initial_guess(1,1) = rotation_matrix[1][1]; initial_guess(1,2) = rotation_matrix[1][2];
        initial_guess(2,0) = rotation_matrix[2][0]; initial_guess(2,1) = rotation_matrix[2][1]; initial_guess(2,2) = rotation_matrix[2][2];
    }

    PointCloudPtr createCorrectedCloud(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg) {
        PointCloudPtr cloud(new PointCloud());
        rclcpp::Time scan_start_time = scan_msg->header.stamp;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
            float range = scan_msg->ranges[i];
            if (std::isinf(range) || std::isnan(range) || range < scan_msg->range_min || range > scan_msg->range_max) continue;
            rclcpp::Time point_time = scan_start_time + rclcpp::Duration::from_seconds(i * scan_msg->time_increment);
            tf2::Quaternion orientation;
            if (!interpolateImu(point_time, orientation)) continue; 
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            tf2::Vector3 point_lidar_frame(range * cos(angle), range * sin(angle), 0.0);
            tf2::Vector3 point_base_frame = tf2::quatRotate(orientation, point_lidar_frame);
            cloud->points.emplace_back(point_base_frame.x(), point_base_frame.y(), point_base_frame.z());
        }
        PointCloudPtr cloud_downsampled(new PointCloud());
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(cloud);
        sor.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
        sor.filter(*cloud_downsampled);
        return cloud_downsampled;
    }

    bool interpolateImu(const rclcpp::Time& time, tf2::Quaternion& output_orientation) {
        std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
        if (imu_buffer_.size() < 2) return false;
        auto it = std::lower_bound(imu_buffer_.begin(), imu_buffer_.end(), time, 
            [](const sensor_msgs::msg::Imu& imu, const rclcpp::Time& t) { return rclcpp::Time(imu.header.stamp) < t; });
        if (it == imu_buffer_.begin() || it == imu_buffer_.end()) return false;
        const auto& after = *it; const auto& before = *(it - 1);
        double dt = (rclcpp::Time(after.header.stamp) - rclcpp::Time(before.header.stamp)).seconds();
        if (dt <= 0.0) return false;
        
        double ratio = (time - rclcpp::Time(before.header.stamp)).seconds() / dt;
        ratio = std::clamp(ratio, 0.0, 1.0);
        
        tf2::Quaternion q_before, q_after;
        tf2::fromMsg(before.orientation, q_before); 
        tf2::fromMsg(after.orientation, q_after);
        output_orientation = q_before.slerp(q_after, ratio).normalized();
        return true;
    }

    // 🔥 수정: Eigen::Matrix4f를 직접 geometry_msgs::Pose로 변환
    void publishOdometry(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = stamp;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;
        
        // 위치 설정
        odom_msg.pose.pose.position.x = pose(0, 3);
        odom_msg.pose.pose.position.y = pose(1, 3);
        odom_msg.pose.pose.position.z = pose(2, 3);
        
        // 회전 설정
        Eigen::Matrix3f rotation_matrix = pose.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation_matrix);
        q.normalize();
        
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        odom_pub_->publish(odom_msg);
    }

    // 🔥 수정: Eigen::Matrix4f를 직접 geometry_msgs::Transform으로 변환
    void publishTf(const rclcpp::Time& stamp, const Eigen::Matrix4f& pose) {
        geometry_msgs::msg::TransformStamped tf_stamped;
        tf_stamped.header.stamp = stamp;
        tf_stamped.header.frame_id = odom_frame_;
        tf_stamped.child_frame_id = base_frame_;
        
        // 위치 설정
        tf_stamped.transform.translation.x = pose(0, 3);
        tf_stamped.transform.translation.y = pose(1, 3);
        tf_stamped.transform.translation.z = pose(2, 3);
        
        // 회전 설정
        Eigen::Matrix3f rotation_matrix = pose.block<3,3>(0,0);
        Eigen::Quaternionf q(rotation_matrix);
        q.normalize();
        
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(tf_stamped);
    }
    
    // 멤버 변수
    std::string odom_frame_, base_frame_;
    double scan_voxel_size_, submap_voxel_size_;
    int submap_keyframes_;
    double keyframe_threshold_dist_, keyframe_threshold_rot_;

    message_filters::Subscriber<sensor_msgs::msg::LaserScan> scan_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::LaserScan, sensor_msgs::msg::Imu>;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    pcl::IterativeClosestPoint<PointType, PointType> icp_;
    PointCloudPtr submap_cloud_;
    std::deque<PointCloudPtr> keyframe_deque_;

    Eigen::Matrix4f current_pose_;
    Eigen::Matrix4f last_keyframe_pose_;
    rclcpp::Time last_scan_time_;
    
    std::deque<sensor_msgs::msg::Imu> imu_buffer_;
    std::mutex imu_buffer_mutex_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP3DOdomNode>());
    rclcpp::shutdown();
    return 0;
}