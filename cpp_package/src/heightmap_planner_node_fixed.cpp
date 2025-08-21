#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "angles/angles.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <deque>

struct HeightCell {
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    float mean_z = 0.0f;
    int point_count = 0;
    bool is_traversable = true;
    float drop_risk = 0.0f;
    rclcpp::Time last_seen;
    
    void addPoint(float z, rclcpp::Time timestamp) {
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
        mean_z = ((mean_z * point_count) + z) / (point_count + 1);
        point_count++;
        last_seen = timestamp;
    }
    
    float getHeightRange() const {
        return max_z - min_z;
    }
};

struct HeightMapFrame {
    rclcpp::Time timestamp;
    std::unordered_map<std::string, HeightCell> cells;
};

class HeightMapPlannerNode : public rclcpp::Node
{
public:
    HeightMapPlannerNode() : Node("heightmap_planner_node"), smoothed_angle_(0.0)
    {
        // Parameters
        this->declare_parameter<bool>("front_view_only", true);
        this->get_parameter("front_view_only", front_view_only_);
        this->declare_parameter<double>("grid_resolution", 0.05);
        this->get_parameter("grid_resolution", grid_resolution_);
        this->declare_parameter<double>("max_range", 3.0);
        this->get_parameter("max_range", max_range_);
        this->declare_parameter<double>("ground_height_tolerance", 0.1);
        this->get_parameter("ground_height_tolerance", ground_height_tolerance_);
        this->declare_parameter<double>("drop_threshold", 0.15);
        this->get_parameter("drop_threshold", drop_threshold_);
        this->declare_parameter<double>("obstacle_height_threshold", 0.2);
        this->get_parameter("obstacle_height_threshold", obstacle_height_threshold_);
        this->declare_parameter<double>("history_duration", 2.0);
        this->get_parameter("history_duration", history_duration_);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/downsampled_cloud", 10, 
            std::bind(&HeightMapPlannerNode::pointCloudCallback, this, std::placeholders::_1));
        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector_heightmap", 10);
        height_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/height_markers", 10);
        
        last_time_ = this->now();
        ground_height_ = 0.0f;
        ground_height_calibrated_ = false;
    }

private:
    void pointCloudCallback(sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        // Filter for front view only
        if (front_view_only_) {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(0.0, max_range_);
            pass.filter(*cloud);
        }
        
        if (!ground_height_calibrated_) {
            calibrateGroundHeight(cloud);
            return;
        }
        
        // Build current height map
        std::unordered_map<std::string, HeightCell> current_map;
        buildHeightMap(cloud, current_map, current_time);
        
        // Update history
        updateHeightMapHistory(current_map, current_time);
        
        // Build accumulated height map
        std::unordered_map<std::string, HeightCell> accumulated_map;
        buildAccumulatedHeightMap(accumulated_map, current_time);
        
        // Analyze traversability
        analyzeTraversability(accumulated_map);
        
        // Plan path
        double safe_angle = planPathWithHeightAwareness(accumulated_map);
        
        // Smooth angle changes
        double angle_diff = angles::shortest_angular_distance(smoothed_angle_, safe_angle);
        double max_angle_change = 0.5 * dt;  // Max 0.5 rad/s
        if (std::abs(angle_diff) > max_angle_change) {
            angle_diff = std::copysign(max_angle_change, angle_diff);
        }
        smoothed_angle_ += angle_diff;
        smoothed_angle_ = angles::normalize_angle(smoothed_angle_);
        
        // Publish results
        publishSafeVector(msg->header, smoothed_angle_);
        publishHeightMapVisualization(msg->header, accumulated_map);
        
        last_time_ = current_time;
    }
    
    void calibrateGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<float> ground_candidates;
        for (const auto& point : cloud->points) {
            if (point.x > 0.5 && point.x < 2.0) {  // 50cm to 2m in front
                ground_candidates.push_back(point.z);
            }
        }
        
        if (ground_candidates.size() > 100) {
            std::sort(ground_candidates.begin(), ground_candidates.end());
            ground_height_ = ground_candidates[ground_candidates.size() / 2];
            ground_height_calibrated_ = true;
            RCLCPP_INFO(this->get_logger(), "Ground height calibrated: %.3f m", ground_height_);
        }
    }
    
    void buildHeightMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       std::unordered_map<std::string, HeightCell>& height_map,
                       rclcpp::Time timestamp)
    {
        for (const auto& point : cloud->points) {
            int grid_x = static_cast<int>(std::floor(point.x / grid_resolution_));
            int grid_y = static_cast<int>(std::floor(point.y / grid_resolution_));
            std::string key = std::to_string(grid_x) + "," + std::to_string(grid_y);
            
            height_map[key].addPoint(point.z, timestamp);
        }
    }
    
    void updateHeightMapHistory(const std::unordered_map<std::string, HeightCell>& current_map,
                               rclcpp::Time current_time)
    {
        HeightMapFrame frame;
        frame.timestamp = current_time;
        frame.cells = current_map;
        height_history_.push_back(frame);
        
        double history_cutoff = (current_time - rclcpp::Duration::from_seconds(history_duration_)).seconds();
        
        while (!height_history_.empty() && 
               height_history_.front().timestamp.seconds() < history_cutoff) {
            height_history_.pop_front();
        }
    }
    
    void buildAccumulatedHeightMap(std::unordered_map<std::string, HeightCell>& accumulated_map,
                                  rclcpp::Time current_time)
    {
        accumulated_map.clear();
        
        for (const auto& frame : height_history_) {
            for (const auto& [key, cell] : frame.cells) {
                if (accumulated_map.find(key) == accumulated_map.end()) {
                    accumulated_map[key] = cell;
                } else {
                    auto& existing_cell = accumulated_map[key];
                    existing_cell.min_z = std::min(existing_cell.min_z, cell.min_z);
                    existing_cell.max_z = std::max(existing_cell.max_z, cell.max_z);
                    
                    float total_points = existing_cell.point_count + cell.point_count;
                    existing_cell.mean_z = (existing_cell.mean_z * existing_cell.point_count + 
                                          cell.mean_z * cell.point_count) / total_points;
                    existing_cell.point_count = total_points;
                    
                    if (cell.last_seen > existing_cell.last_seen) {
                        existing_cell.last_seen = cell.last_seen;
                    }
                }
            }
        }
    }
    
    void analyzeTraversability(std::unordered_map<std::string, HeightCell>& height_map)
    {
        for (auto& [key, cell] : height_map) {
            if (cell.point_count == 0) continue;
            
            float height_diff_from_ground = cell.min_z - ground_height_;
            
            if (height_diff_from_ground < -drop_threshold_) {
                cell.is_traversable = false;
                cell.drop_risk = std::min(1.0f, static_cast<float>(std::abs(height_diff_from_ground) / drop_threshold_));
            }
            else if (cell.max_z - ground_height_ > obstacle_height_threshold_) {
                cell.is_traversable = false;
                cell.drop_risk = 0.0f;
            }
            else if (cell.getHeightRange() > ground_height_tolerance_) {
                cell.is_traversable = false;
                cell.drop_risk = 0.2f;  // Medium risk for uneven terrain
            }
            else {
                cell.is_traversable = true;
                cell.drop_risk = 0.0f;
            }
        }
    }
    
    double planPathWithHeightAwareness(const std::unordered_map<std::string, HeightCell>& height_map)
    {
        const int num_rays = 50;
        std::vector<double> scores(num_rays, 0.0);
        
        for (int i = 0; i < num_rays; ++i) {
            double ray_angle = (i * (M_PI / (num_rays - 1))) - (M_PI / 2.0);
            Eigen::Vector2f ray_dir(cos(ray_angle), sin(ray_angle));
            
            double total_clearance = max_range_;
            double total_risk_penalty = 0.0;
            int cells_checked = 0;
            double ray_clearance = 0.2;  // 20cm clearance
            
            for (double dist = 0.2; dist < max_range_; dist += grid_resolution_) {
                Eigen::Vector2f ray_point = ray_dir * dist;
                
                for (double offset = -ray_clearance; offset <= ray_clearance; offset += grid_resolution_) {
                    Eigen::Vector2f perp_dir(-ray_dir.y(), ray_dir.x());
                    Eigen::Vector2f check_point = ray_point + perp_dir * offset;
                    
                    int grid_x = static_cast<int>(std::floor(check_point.x() / grid_resolution_));
                    int grid_y = static_cast<int>(std::floor(check_point.y() / grid_resolution_));
                    std::string key = std::to_string(grid_x) + "," + std::to_string(grid_y);
                    
                    auto it = height_map.find(key);
                    if (it != height_map.end()) {
                        const HeightCell& cell = it->second;
                        
                        if (!cell.is_traversable) {
                            total_clearance = std::min(total_clearance, dist);
                        }
                        
                        total_risk_penalty += cell.drop_risk;
                        cells_checked++;
                    }
                }
            }
            
            double avg_risk = cells_checked > 0 ? total_risk_penalty / cells_checked : 0.0;
            scores[i] = total_clearance * (1.0 - avg_risk * 2.0);
            
            if (i > 0 && i < num_rays - 1) {
                double neighbor_boost = 0.1 * (scores[i-1] + scores[i+1]);
                scores[i] += neighbor_boost;
            }
        }
        
        int best_ray_idx = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
        double ideal_angle = (best_ray_idx * (M_PI / (num_rays - 1))) - (M_PI / 2.0);
        
        return ideal_angle;
    }
    
    void publishSafeVector(const std_msgs::msg::Header& header, double angle)
    {
        geometry_msgs::msg::Vector3Stamped safe_vector_msg;
        safe_vector_msg.header = header;
        safe_vector_msg.vector.x = cos(angle);
        safe_vector_msg.vector.y = sin(angle);
        safe_vector_msg.vector.z = 0;
        vector_publisher_->publish(safe_vector_msg);
    }
    
    void publishHeightMapVisualization(const std_msgs::msg::Header& header,
                                      const std::unordered_map<std::string, HeightCell>& height_map)
    {
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        
        for (const auto& [key, cell] : height_map) {
            if (cell.point_count == 0) continue;
            
            size_t comma_pos = key.find(',');
            int grid_x = std::stoi(key.substr(0, comma_pos));
            int grid_y = std::stoi(key.substr(comma_pos + 1));
            
            double world_x = (grid_x + 0.5) * grid_resolution_;
            double world_y = (grid_y + 0.5) * grid_resolution_;
            
            // Grid cell visualization as cubes
            visualization_msgs::msg::Marker grid_marker;
            grid_marker.header = header;
            grid_marker.ns = "height_grid";
            grid_marker.id = marker_id++;
            grid_marker.type = visualization_msgs::msg::Marker::CUBE;
            grid_marker.action = visualization_msgs::msg::Marker::ADD;
            grid_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
            
            grid_marker.pose.position.x = world_x;
            grid_marker.pose.position.y = world_y;
            grid_marker.pose.position.z = ground_height_ + (cell.max_z - cell.min_z) / 2.0;
            grid_marker.pose.orientation.w = 1.0;
            
            grid_marker.scale.x = grid_resolution_ * 0.9;
            grid_marker.scale.y = grid_resolution_ * 0.9;
            
            double height_range = cell.max_z - cell.min_z;
            if (height_range < 0.01) height_range = 0.01;
            grid_marker.scale.z = height_range;
            
            if (!cell.is_traversable) {
                if (cell.drop_risk > 0.5) {
                    grid_marker.color.r = 0.9; grid_marker.color.g = 0.0; grid_marker.color.b = 0.0;
                } else {
                    grid_marker.color.r = 1.0; grid_marker.color.g = 0.4; grid_marker.color.b = 0.0;
                }
            } else {
                double height_offset = cell.mean_z - ground_height_;
                if (std::abs(height_offset) < 0.05) {
                    grid_marker.color.r = 0.0; grid_marker.color.g = 0.8; grid_marker.color.b = 0.2;
                } else {
                    grid_marker.color.r = 0.2; grid_marker.color.g = 0.6; grid_marker.color.b = 0.3;
                }
            }
            grid_marker.color.a = 0.7;
            
            markers.markers.push_back(grid_marker);
        }
        
        height_publisher_->publish(markers);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr height_publisher_;
    
    bool front_view_only_;
    double grid_resolution_;
    double max_range_;
    double ground_height_tolerance_;
    double drop_threshold_;
    double obstacle_height_threshold_;
    double history_duration_;
    
    double smoothed_angle_;
    rclcpp::Time last_time_;
    
    float ground_height_;
    bool ground_height_calibrated_;
    
    std::deque<HeightMapFrame> height_history_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightMapPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
