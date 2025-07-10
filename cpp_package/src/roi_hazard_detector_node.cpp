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

class ROIHazardDetectorNode : public rclcpp::Node
{
public:
    ROIHazardDetectorNode() : Node("roi_hazard_detector_node")
    {
        // Parameters
        this->declare_parameter<double>("roi.x_m", 2.0); // Forward distance
        this->declare_parameter<double>("roi.y_m", 1.5); // Width
        this->declare_parameter<double>("roi.z_m", 1.8); // Height from the ground plane
        this->declare_parameter<int>("roi.x_cells", 3);
        this->declare_parameter<int>("roi.y_cells", 3);
        this->declare_parameter<int>("roi.z_cells", 3);
        this->declare_parameter<int>("cell_activation_threshold", 10); // Min points to consider a cell active
        this->declare_parameter<double>("stop_distance_m", 0.8);

        // Publishers and Subscribers
        haptic_command_pub_ = this->create_publisher<std_msgs::msg::String>("/haptic_command", 10);
        
        obstacle_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/obstacle_points", 10, std::bind(&ROIHazardDetectorNode::obstacle_callback, this, std::placeholders::_1));
            
        plane_coeffs_sub_ = this->create_subscription<pcl_msgs::msg::ModelCoefficients>(
            "/locked_plane_coefficients", 10, std::bind(&ROIHazardDetectorNode::plane_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "ROI Hazard Detector Node started.");
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

        // Get ROI parameters
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

        // Initialize cell data structure
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cells(x_cells * y_cells * z_cells);
        for(auto& cell : cells) {
            cell = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        }

        // Plane coefficients for distance calculation
        double a = locked_plane_coefficients_->values[0];
        double b = locked_plane_coefficients_->values[1];
        double c = locked_plane_coefficients_->values[2];
        double d = locked_plane_coefficients_->values[3];

        // Classify points into cells
        for (const auto& point : cloud->points) {
            double dist_to_plane = std::abs(a * point.x + b * point.y + c * point.z + d);

            // Check if point is within the overall ROI box relative to the plane
            if (point.x > 0 && point.x < roi_x && std::abs(point.y) < roi_y / 2.0 && dist_to_plane < roi_z) {
                int x_idx = static_cast<int>(point.x / cell_size_x);
                int y_idx = static_cast<int>((point.y + roi_y / 2.0) / cell_size_y);
                int z_idx = static_cast<int>(dist_to_plane / cell_size_z);

                if (x_idx < x_cells && y_idx < y_cells && z_idx < z_cells) {
                    int cell_index = z_idx * (x_cells * y_cells) + x_idx * y_cells + y_idx;
                    cells[cell_index]->points.push_back(point);
                }
            }
        }

        // --- Analyze cells and generate command ---
        std::string command = "CLEAR"; // Default command

        // Priority 1: Check for immediate obstacles that require stopping
        double stop_dist = this->get_parameter("stop_distance_m").as_double();
        bool stop_triggered = false;
        for (int z = 0; z < z_cells; ++z) {
            for (int x = 0; x < x_cells; ++x) {
                int center_y = y_cells / 2;
                int cell_idx = z * (x_cells * y_cells) + x * y_cells + center_y;
                if (cells[cell_idx]->points.size() > (size_t)activation_threshold) {
                    for (const auto& pt : cells[cell_idx]->points) {
                        if (pt.x < stop_dist) {
                            command = "STOP";
                            stop_triggered = true;
                            break;
                        }
                    }
                }
                if(stop_triggered) break;
            }
            if(stop_triggered) break;
        }

        // Priority 2: Check for steps if not stopping
        if (!stop_triggered) {
            int front_x = 0; // Closest row of cells
            int left_y = 0;
            int center_y = 1;
            int right_y = 2;
            
            int bottom_z = 0;

            int left_step_idx = bottom_z * (x_cells * y_cells) + front_x * y_cells + left_y;
            int center_step_idx = bottom_z * (x_cells * y_cells) + front_x * y_cells + center_y;
            int right_step_idx = bottom_z * (x_cells * y_cells) + front_x * y_cells + right_y;

            if (cells[center_step_idx]->points.size() > (size_t)activation_threshold) {
                command = "FRONT_STEP_UP";
            } else if (cells[left_step_idx]->points.size() > (size_t)activation_threshold) {
                command = "LEFT_STEP_UP";
            } else if (cells[right_step_idx]->points.size() > (size_t)activation_threshold) {
                command = "RIGHT_STEP_UP";
            }
        }
        
        // TODO: Add logic for TURN_LEFT/RIGHT based on middle layer obstacles
        // This is a placeholder for more complex logic
        
        publish_command(command);
    }

    void publish_command(const std::string& command) {
        if (last_command_ != command || command != "CLEAR") { // Publish non-clear commands every time
            std_msgs::msg::String command_msg;
            command_msg.data = command;
            haptic_command_pub_->publish(command_msg);
            last_command_ = command;
            if (command != "CLEAR") {
                RCLCPP_INFO(this->get_logger(), "Published command: %s", command.c_str());
            }
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<pcl_msgs::msg::ModelCoefficients>::SharedPtr plane_coeffs_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr haptic_command_pub_;
    
    pcl_msgs::msg::ModelCoefficients::SharedPtr locked_plane_coefficients_;
    std::string last_command_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ROIHazardDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
