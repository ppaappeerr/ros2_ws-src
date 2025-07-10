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

class HazardDetectorNode : public rclcpp::Node
{
public:
    HazardDetectorNode() : Node("hazard_detector_node")
    {
        // Parameters
        this->declare_parameter<double>("min_step_height", 0.05);
        this->declare_parameter<double>("max_step_height", 0.25);
        this->declare_parameter<int>("min_step_points", 20);
        this->declare_parameter<double>("dropoff_roi_forward_m", 1.2);
        this->declare_parameter<double>("dropoff_roi_width_m", 1.0);
        this->declare_parameter<double>("grid_resolution_m", 0.1);
        this->declare_parameter<int>("empty_cell_threshold", 5);
        this->declare_parameter<double>("obstacle_roi_forward_m", 2.0);
        this->declare_parameter<double>("obstacle_roi_width_m", 1.5);
        this->declare_parameter<double>("stop_distance_m", 1.0);

        // Publishers and Subscribers
        haptic_command_pub_ = this->create_publisher<std_msgs::msg::String>("/haptic_command", 10);
        
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacle_points", 10, std::bind(&HazardDetectorNode::obstacle_callback, this, std::placeholders::_1));
            
        plane_coeffs_sub_ = this->create_subscription<pcl_msgs::msg::ModelCoefficients>(
            "/locked_plane_coefficients", 10, std::bind(&HazardDetectorNode::plane_callback, this, std::placeholders::_1));

        ground_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/stable_ground_points", 10, std::bind(&HazardDetectorNode::ground_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Hazard Detector Node started.");
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
        if (!locked_plane_coefficients_ || locked_plane_coefficients_->values.empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // --- 1. Step Detection (Positive Obstacle) ---
        if (detect_step(cloud)) {
            return; // Prioritize step warning
        }

        // --- 2. General Obstacle Avoidance ---
        detect_general_obstacles(cloud);
    }

    bool detect_step(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<int> step_point_indices;
        double a = locked_plane_coefficients_->values[0];
        double b = locked_plane_coefficients_->values[1];
        double c = locked_plane_coefficients_->values[2];
        double d = locked_plane_coefficients_->values[3];
        
        double roi_forward = this->get_parameter("obstacle_roi_forward_m").as_double();
        double roi_half_width = this->get_parameter("obstacle_roi_width_m").as_double() / 2.0;
        double min_height = this->get_parameter("min_step_height").as_double();
        double max_height = this->get_parameter("max_step_height").as_double();

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& point = cloud->points[i];
            if (point.x > 0.2 && point.x < roi_forward && std::abs(point.y) < roi_half_width) {
                double distance = std::abs(a * point.x + b * point.y + c * point.z + d);
                if (distance > min_height && distance < max_height) {
                    step_point_indices.push_back(i);
                }
            }
        }

        if (step_point_indices.size() > (size_t)this->get_parameter("min_step_points").as_int()) {
            publish_command("STEP_UP");
            return true;
        }
        return false;
    }

    void detect_general_obstacles(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        double roi_forward = this->get_parameter("obstacle_roi_forward_m").as_double();
        double roi_width = this->get_parameter("obstacle_roi_width_m").as_double();
        double stop_dist = this->get_parameter("stop_distance_m").as_double();

        double zone_width = roi_width / 3.0; // Left, Center, Right
        double min_dist_left = std::numeric_limits<double>::max();
        double min_dist_center = std::numeric_limits<double>::max();
        double min_dist_right = std::numeric_limits<double>::max();

        for (const auto& point : cloud->points) {
            if (point.x > 0.1 && point.x < roi_forward) {
                double dist = std::sqrt(point.x * point.x + point.y * point.y);
                
                // Center Zone
                if (std::abs(point.y) < zone_width / 2.0) {
                    if (dist < min_dist_center) min_dist_center = dist;
                } 
                // Left Zone
                else if (point.y > 0) {
                    if (dist < min_dist_left) min_dist_left = dist;
                }
                // Right Zone
                else {
                    if (dist < min_dist_right) min_dist_right = dist;
                }
            }
        }

        // Decision Logic
        if (min_dist_center < stop_dist) {
            publish_command("STOP");
        } else if (min_dist_left < roi_forward && min_dist_right >= roi_forward) {
            publish_command("TURN_RIGHT");
        } else if (min_dist_right < roi_forward && min_dist_left >= roi_forward) {
            publish_command("TURN_LEFT");
        } else {
            publish_command("CLEAR");
        }
    }

    void ground_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!locked_plane_coefficients_ || locked_plane_coefficients_->values.empty()) return;

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *ground_cloud);

        double roi_forward = this->get_parameter("dropoff_roi_forward_m").as_double();
        double roi_width = this->get_parameter("dropoff_roi_width_m").as_double();
        double grid_res = this->get_parameter("grid_resolution_m").as_double();

        int grid_x_size = static_cast<int>(roi_forward / grid_res);
        int grid_y_size = static_cast<int>(roi_width / grid_res);
        
        std::map<int, int> grid_counts;

        for (const auto& point : ground_cloud->points) {
            if (point.x > 0 && point.x < roi_forward && std::abs(point.y) < roi_width / 2.0) {
                int grid_x = static_cast<int>(point.x / grid_res);
                int grid_y = static_cast<int>((point.y + roi_width / 2.0) / grid_res);
                grid_counts[grid_x * grid_y_size + grid_y]++;
            }
        }

        int empty_cell_threshold = this->get_parameter("empty_cell_threshold").as_int();
        for (int i = 1; i < grid_x_size; ++i) {
            int empty_cells_in_row = 0;
            for (int j = 0; j < grid_y_size; ++j) {
                if (grid_counts.find(i * grid_y_size + j) == grid_counts.end()) {
                    empty_cells_in_row++;
                }
            }
            if (empty_cells_in_row >= empty_cell_threshold) {
                publish_command("DROPOFF_AHEAD");
                return;
            }
        }
    }

    void publish_command(const std::string& command) {
        // To avoid spamming, only publish if the command changes
        if (last_command_ != command) {
            std_msgs::msg::String command_msg;
            command_msg.data = command;
            haptic_command_pub_->publish(command_msg);
            last_command_ = command;
            RCLCPP_INFO(this->get_logger(), "Published command: %s", command.c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<pcl_msgs::msg::ModelCoefficients>::SharedPtr plane_coeffs_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr haptic_command_pub_;
    
    pcl_msgs::msg::ModelCoefficients::SharedPtr locked_plane_coefficients_;
    std::string last_command_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HazardDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
