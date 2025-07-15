#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_msgs/msg/model_coefficients.hpp"
#include "std_msgs/msg/string.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <vector>
#include <map>
#include <limits>

class UserCentricROIDetector : public rclcpp::Node
{
public:
    UserCentricROIDetector() : Node("user_centric_roi_detector")
    {
        // Parameters
        this->declare_parameter<double>("roi.x_m", 2.0); // Forward distance from user
        this->declare_parameter<double>("roi.y_m", 1.5); // Width, centered on user
        this->declare_parameter<double>("roi.z_m", 1.8); // Height from the ground plane
        this->declare_parameter<int>("roi.x_cells", 3);  // 3: NEAR, MID, FAR
        this->declare_parameter<int>("roi.y_cells", 3);  // 3: LEFT, CENTER, RIGHT
        this->declare_parameter<int>("roi.z_cells", 3);  // 3: LOW, MID, HIGH
        this->declare_parameter<int>("cell_activation_threshold", 10);
        this->declare_parameter<double>("stop_distance_m", 0.8);

        // Publishers and Subscribers
        haptic_command_pub_ = this->create_publisher<std_msgs::msg::String>("/haptic_command", 10);
        
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacle_points", 10, std::bind(&UserCentricROIDetector::obstacle_callback, this, std::placeholders::_1));
            
        plane_coeffs_sub_ = this->create_subscription<pcl_msgs::msg::ModelCoefficients>(
            "/locked_plane_coefficients", 10, std::bind(&UserCentricROIDetector::plane_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "User-Centric ROI Detector Node started.");
    }

private:
    void plane_callback(const pcl_msgs::msg::ModelCoefficients::SharedPtr msg)
    {
        if (msg->values.size() == 4) {
            locked_plane_coefficients_ = msg;
        }
    }

    void obstacle_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!locked_plane_coefficients_ || locked_plane_coefficients_->values.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for locked plane coefficients...");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // --- 1. Pre-filter for front hemisphere ---
        pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(const auto& point : cloud->points) {
            if (point.x > 0) {
                front_cloud->points.push_back(point);
            }
        }
        if (front_cloud->empty()) {
            publish_command("OK"); // 변경: 앞쪽에 포인트가 없으면 OK 상태
            return;
        }

        // --- 2. Classify points into user-centric ROI cells ---
        double roi_x = this->get_parameter("roi.x_m").as_double();
        double roi_y = this->get_parameter("roi.y_m").as_double();
        double roi_z = this->get_parameter("roi.z_m").as_double();
        int x_cells = this->get_parameter("roi.x_cells").as_int();
        int y_cells = this->get_parameter("roi.y_cells").as_int();
        int z_cells = this->get_parameter("roi.z_cells").as_int();
        int activation_threshold = this->get_parameter("cell_activation_threshold").as_int();

        double cell_size_x = roi_x / x_cells;
        double cell_size_y = roi_y / y_cells;
        double cell_size_z = roi_z / z_cells;

        // 3D ROI 텐서 초기화 (x: distance, y: horizontal, z: vertical)
        std::vector<std::vector<std::vector<int>>> roi_tensor(x_cells, 
            std::vector<std::vector<int>>(y_cells, 
                std::vector<int>(z_cells, 0)));

        double a = locked_plane_coefficients_->values[0];
        double b = locked_plane_coefficients_->values[1];
        double c = locked_plane_coefficients_->values[2];
        double d = locked_plane_coefficients_->values[3];

        // 각 포인트를 ROI 텐서에 할당
        for (const auto& point : front_cloud->points) {
            double dist_to_plane = std::abs(a * point.x + b * point.y + c * point.z + d);

            if (point.x < roi_x && std::abs(point.y) < roi_y / 2.0 && dist_to_plane < roi_z) {
                int x_idx = static_cast<int>(point.x / cell_size_x);
                int y_idx = static_cast<int>((point.y + roi_y / 2.0) / cell_size_y);
                int z_idx = static_cast<int>(dist_to_plane / cell_size_z);

                if (x_idx >= 0 && x_idx < x_cells && 
                    y_idx >= 0 && y_idx < y_cells && 
                    z_idx >= 0 && z_idx < z_cells) {
                    roi_tensor[x_idx][y_idx][z_idx]++;
                }
            }
        }

        // --- 3. 우선순위 기반 명령어 생성 ---
        std::string command = "OK"; // 변경: 기본값은 OK로 설정

        // 우선순위에 따른 ROI 텐서 분석
        bool obstacle_detected = false;
        
        // 정면 가까운 영역부터 검사 (높은 우선순위)
        if (roi_tensor[0][1][2] > activation_threshold) {
            command = "FRONT_HIGH";
            obstacle_detected = true;
        }
        else if (roi_tensor[0][1][1] > activation_threshold) {
            command = "FRONT_MID";
            obstacle_detected = true;
        }
        else if (roi_tensor[0][1][0] > activation_threshold) {
            command = "FRONT_LOW";
            obstacle_detected = true;
        }
        // 왼쪽 영역 검사
        else if (roi_tensor[0][0][2] > activation_threshold) {
            command = "LEFT_HIGH";
            obstacle_detected = true;
        }
        else if (roi_tensor[0][0][1] > activation_threshold) {
            command = "LEFT_MID";
            obstacle_detected = true;
        }
        else if (roi_tensor[0][0][0] > activation_threshold) {
            command = "LEFT_LOW";
            obstacle_detected = true;
        }
        // 오른쪽 영역 검사
        else if (roi_tensor[0][2][2] > activation_threshold) {
            command = "RIGHT_HIGH";
            obstacle_detected = true;
        }
        else if (roi_tensor[0][2][1] > activation_threshold) {
            command = "RIGHT_MID";
            obstacle_detected = true;
        }
        else if (roi_tensor[0][2][0] > activation_threshold) {
            command = "RIGHT_LOW";
            obstacle_detected = true;
        }
        // 중거리 영역 검사 (중간 우선순위)
        else if (roi_tensor[1][1][1] > activation_threshold) {
            command = "FRONT_MID";
            obstacle_detected = true;
        }
        else if (roi_tensor[1][0][1] > activation_threshold) {
            command = "LEFT_MID";
            obstacle_detected = true;
        }
        else if (roi_tensor[1][2][1] > activation_threshold) {
            command = "RIGHT_MID";
            obstacle_detected = true;
        }
        // 원거리 영역 검사 (낮은 우선순위)
        else if (roi_tensor[2][1][1] > activation_threshold) {
            command = "FRONT_LOW"; // 원거리는 낮은 강도로 알림
            obstacle_detected = true;
        }
        
        // 장애물이 없는 경우 OK로 설정 (변경됨)
        if (!obstacle_detected) {
            command = "OK"; // 안전한 경우에는 OK 명령
        }

        publish_command(command);
    }

    void publish_command(const std::string& command) {
        if (last_command_ != command) {
            std_msgs::msg::String command_msg;
            command_msg.data = command;
            haptic_command_pub_->publish(command_msg);
            RCLCPP_INFO(this->get_logger(), "Published command: %s", command.c_str());
            last_command_ = command;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<pcl_msgs::msg::ModelCoefficients>::SharedPtr plane_coeffs_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr haptic_command_pub_;
    
    pcl_msgs::msg::ModelCoefficients::SharedPtr locked_plane_coefficients_;
    std::string last_command_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UserCentricROIDetector>());
    rclcpp::shutdown();
    return 0;
}