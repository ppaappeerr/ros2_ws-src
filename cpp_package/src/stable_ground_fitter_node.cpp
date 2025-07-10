#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/distances.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class StableGroundFitterNode : public rclcpp::Node
{
public:
    StableGroundFitterNode() : Node("stable_ground_fitter_node"), plane_locked_(false), ransac_count_(0)
    {
        // Parameters
        this->declare_parameter<std::string>("input_topic", "/sweep_cloud_cpp");
        this->declare_parameter<std::string>("refined_ground_topic", "/refined_ground_points");
        this->declare_parameter<std::string>("obstacle_topic", "/obstacle_points");
        this->declare_parameter<std::string>("marker_topic", "/ground_plane_marker");
        this->declare_parameter<double>("ground_height_threshold_min", -1.7);
        this->declare_parameter<double>("ground_height_threshold_max", -1.0);
        this->declare_parameter<double>("voxel_leaf_size", 0.05);
        this->declare_parameter<double>("ransac_distance_threshold", 0.05);
        this->declare_parameter<int>("ransac_lock_count", 50);
        this->declare_parameter<bool>("reset_plane_detection", false);
        this->declare_parameter<double>("marker_scale_x", 10.0);
        this->declare_parameter<double>("marker_scale_y", 10.0);
        this->declare_parameter<double>("marker_scale_z", 0.01);
        this->declare_parameter<double>("marker_alpha", 0.5);
        this->declare_parameter<double>("marker_locked_r", 1.0);
        this->declare_parameter<double>("marker_locked_g", 0.0);
        this->declare_parameter<double>("marker_locked_b", 0.0);
        this->declare_parameter<double>("marker_learning_r", 0.0);
        this->declare_parameter<double>("marker_learning_g", 1.0);
        this->declare_parameter<double>("marker_learning_b", 0.0);

        this->get_parameter("input_topic", input_topic_);
        this->get_parameter("refined_ground_topic", refined_ground_topic_);
        this->get_parameter("obstacle_topic", obstacle_topic_);
        this->get_parameter("marker_topic", marker_topic_);
        this->get_parameter("ground_height_threshold_min", ground_height_threshold_min_);
        this->get_parameter("ground_height_threshold_max", ground_height_threshold_max_);
        this->get_parameter("voxel_leaf_size", voxel_leaf_size_);
        this->get_parameter("ransac_distance_threshold", ransac_distance_threshold_);
        this->get_parameter("ransac_lock_count", ransac_lock_count_);

        // 파라미터 콜백 등록
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&StableGroundFitterNode::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Stable Ground Fitter Node started.");
        RCLCPP_INFO(this->get_logger(), "RANSAC lock count: %d", ransac_lock_count_);

        // Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, 10, std::bind(&StableGroundFitterNode::cloud_callback, this, std::placeholders::_1));

        // Publishers
        ground_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(refined_ground_topic_, 10);
        obstacle_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(obstacle_topic_, 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, 10);
        plane_coeffs_pub_ = this->create_publisher<pcl_msgs::msg::ModelCoefficients>("/locked_plane_coefficients", 10);

        accumulated_candidates_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        final_ground_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        locked_plane_coefficients_ = pcl::make_shared<pcl::ModelCoefficients>();
        
        // 상태 표시 타이머 (1초마다 현재 RANSAC 카운트 출력)
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&StableGroundFitterNode::status_callback, this));
    }

private:
    // 파라미터 변경 콜백 함수
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            if (param.get_name() == "ransac_lock_count")
            {
                int new_count = param.as_int();
                if (new_count < 1)
                {
                    result.successful = false;
                    result.reason = "ransac_lock_count must be >= 1";
                    return result;
                }
                
                ransac_lock_count_ = new_count;
                RCLCPP_INFO(this->get_logger(), "Updated RANSAC lock count to %d", ransac_lock_count_);
                
                // 이미 잠긴 상태라면 새 카운트가 현재 카운트보다 크면 잠금 해제
                if (plane_locked_ && ransac_count_ <= ransac_lock_count_)
                {
                    plane_locked_ = false;
                    RCLCPP_INFO(this->get_logger(), 
                        "Unlocked plane detection: current count %d < new threshold %d", 
                        ransac_count_, ransac_lock_count_);
                    
                    // 다시 누적 클라우드 초기화
                    accumulated_candidates_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                }
            }
            else if (param.get_name() == "reset_plane_detection" && param.as_bool())
            {
                // 평면 감지 리셋 기능
                plane_locked_ = false;
                ransac_count_ = 0;
                accumulated_candidates_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                final_ground_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
                locked_plane_coefficients_ = pcl::make_shared<pcl::ModelCoefficients>();
                
                RCLCPP_INFO(this->get_logger(), "Reset plane detection state!");
                
                // reset_plane_detection 파라미터를 다시 false로 설정
                this->set_parameter(rclcpp::Parameter("reset_plane_detection", false));
            }
            else if (param.get_name() == "ransac_distance_threshold")
            {
                ransac_distance_threshold_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated RANSAC distance threshold to %.3f", ransac_distance_threshold_);
            }
            else if (param.get_name() == "ground_height_threshold_min")
            {
                ground_height_threshold_min_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated ground height min to %.3f", ground_height_threshold_min_);
            }
            else if (param.get_name() == "ground_height_threshold_max")
            {
                ground_height_threshold_max_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated ground height max to %.3f", ground_height_threshold_max_);
            }
            else if (param.get_name() == "voxel_leaf_size")
            {
                voxel_leaf_size_ = param.as_double();
                RCLCPP_INFO(this->get_logger(), "Updated voxel leaf size to %.3f", voxel_leaf_size_);
            }
        }

        return result;
    }
    
    // 상태 표시 콜백 함수
    void status_callback()
    {
        if (!plane_locked_)
        {
            RCLCPP_DEBUG(this->get_logger(), 
                "Plane detection progress: %d/%d iterations", 
                ransac_count_, ransac_lock_count_);
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);

        if (input_cloud->empty()) return;

        // --- Obstacle Extraction (from original cloud) ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_obs;
        pass_obs.setInputCloud(input_cloud);
        pass_obs.setFilterFieldName("z");
        pass_obs.setFilterLimits(ground_height_threshold_min_, ground_height_threshold_max_);
        pass_obs.setNegative(true);
        pass_obs.filter(*obstacle_cloud);
        
        if (!obstacle_cloud->empty()) {
            sensor_msgs::msg::PointCloud2 obstacle_msg;
            pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
            obstacle_msg.header = msg->header;
            obstacle_publisher_->publish(obstacle_msg);
        }

        // --- Ground Candidate Extraction ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_candidates(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass_ground;
        pass_ground.setInputCloud(input_cloud);
        pass_ground.setFilterFieldName("z");
        pass_ground.setFilterLimits(ground_height_threshold_min_, ground_height_threshold_max_);
        pass_ground.filter(*ground_candidates);

        if (ground_candidates->empty()) return;

        if (!plane_locked_)
        {
            // --- RANSAC Phase ---
            *accumulated_candidates_ += *ground_candidates;
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(accumulated_candidates_);
            voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_grid.filter(*accumulated_candidates_);

            if (accumulated_candidates_->points.size() < 10) return;

            pcl::ModelCoefficients::Ptr current_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ransac_distance_threshold_);
            seg.setInputCloud(accumulated_candidates_);
            seg.segment(*inliers, *current_coefficients);

            if (inliers->indices.size() == 0) return;

            *locked_plane_coefficients_ = *current_coefficients;
            ransac_count_++;

            // Extract inliers and add to the final cloud
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(accumulated_candidates_);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*final_ground_cloud_); // Start building the final cloud

            if (ransac_count_ >= ransac_lock_count_)
            {
                plane_locked_ = true;
                RCLCPP_INFO(this->get_logger(), "Ground plane locked with %d iterations.", ransac_count_);
                accumulated_candidates_.reset(); // No longer needed
            }
        }
        else
        {
            // --- Plane Locked Phase: Accumulate points ---
            for (const auto& point : ground_candidates->points)
            {
                double distance = std::abs(
                    locked_plane_coefficients_->values[0] * point.x +
                    locked_plane_coefficients_->values[1] * point.y +
                    locked_plane_coefficients_->values[2] * point.z +
                    locked_plane_coefficients_->values[3]
                );

                if (distance < ransac_distance_threshold_)
                {
                    final_ground_cloud_->points.push_back(point);
                }
            }
        }

        // --- Voxelize and Publish Final Ground Cloud ---
        if (!final_ground_cloud_->points.empty()) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_final;
            voxel_grid_final.setInputCloud(final_ground_cloud_);
            voxel_grid_final.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_grid_final.filter(*final_ground_cloud_);

            sensor_msgs::msg::PointCloud2 ground_msg;
            pcl::toROSMsg(*final_ground_cloud_, ground_msg);
            ground_msg.header = msg->header;
            ground_publisher_->publish(ground_msg);
        }

        // --- Publish Plane Marker ---
        if (!locked_plane_coefficients_->values.empty()) {
            publish_plane_marker(msg->header, *locked_plane_coefficients_);
        }
    }

    void publish_plane_marker(const std_msgs::msg::Header& header, const pcl::ModelCoefficients& coeffs)
    {
        if (plane_locked_) {
            auto coeffs_msg = std::make_unique<pcl_msgs::msg::ModelCoefficients>();
            coeffs_msg->header = header;
            for(const auto& value : coeffs.values)
                coeffs_msg->values.push_back(value);
            plane_coeffs_pub_->publish(std::move(coeffs_msg));
        }

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "ground_plane";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        tf2::Vector3 normal(coeffs.values[0], coeffs.values[1], coeffs.values[2]);
        if (normal.length2() < 1e-6) return;
        normal.normalize();

        tf2::Vector3 up(0, 0, 1);
        tf2::Quaternion q;
        tf2::Vector3 axis = normal.cross(up);
        if (axis.length2() < 1e-6) {
            q.setRPY(0, (normal.z() > 0 ? 0 : M_PI), 0);
        } else {
            q.setRotation(axis.normalized(), normal.angle(up));
        }
        
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        if (std::abs(coeffs.values[2]) < 1e-6) return;
        marker.pose.position.z = -coeffs.values[3] / coeffs.values[2];

        // 파라미터에서 마커 속성 불러오기
        double scale_x = this->get_parameter("marker_scale_x").as_double();
        double scale_y = this->get_parameter("marker_scale_y").as_double();
        double scale_z = this->get_parameter("marker_scale_z").as_double();
        double alpha = this->get_parameter("marker_alpha").as_double();
        
        double locked_r = this->get_parameter("marker_locked_r").as_double();
        double locked_g = this->get_parameter("marker_locked_g").as_double();
        double locked_b = this->get_parameter("marker_locked_b").as_double();
        
        double learning_r = this->get_parameter("marker_learning_r").as_double();
        double learning_g = this->get_parameter("marker_learning_g").as_double();
        double learning_b = this->get_parameter("marker_learning_b").as_double();
        
        // 마커 속성 설정
        marker.scale.x = scale_x;
        marker.scale.y = scale_y;
        marker.scale.z = scale_z;
        
        marker.color.a = alpha;
        marker.color.r = plane_locked_ ? locked_r : learning_r;
        marker.color.g = plane_locked_ ? locked_g : learning_g;
        marker.color.b = plane_locked_ ? locked_b : learning_b;

        marker_publisher_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    rclcpp::Publisher<pcl_msgs::msg::ModelCoefficients>::SharedPtr plane_coeffs_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_candidates_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr final_ground_cloud_;
    pcl::ModelCoefficients::Ptr locked_plane_coefficients_;
    bool plane_locked_;
    int ransac_count_;
    int ransac_lock_count_;

    std::string input_topic_, refined_ground_topic_, obstacle_topic_, marker_topic_;
    double ground_height_threshold_min_, ground_height_threshold_max_, voxel_leaf_size_, ransac_distance_threshold_;
    
    // 파라미터 콜백 핸들
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StableGroundFitterNode>());
    rclcpp::shutdown();
    return 0;
}
