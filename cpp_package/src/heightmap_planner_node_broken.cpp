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
    float drop_risk = 0.0f;  // 낙차 위험도 (0.0 = 안전, 1.0 = 매우 위험)
    rclcpp::Time last_seen;  // 마지막으로 관찰된 시간
    
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

// 시간 누적용 높이맵 프레임
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
        this->declare_parameter<double>("grid_resolution", 0.05);  // 5cm cells - 성능과 정밀도 균형
        this->get_parameter("grid_resolution", grid_resolution_);
        this->declare_parameter<double>("max_range", 5.0);
        this->get_parameter("max_range", max_range_);
        this->declare_parameter<double>("ground_height_tolerance", 0.15);  // 15cm tolerance
        this->get_parameter("ground_height_tolerance", ground_height_tolerance_);
        this->declare_parameter<double>("drop_threshold", 0.3);  // 30cm drop = dangerous
        this->get_parameter("drop_threshold", drop_threshold_);
        this->declare_parameter<double>("obstacle_height_threshold", 0.2);  // 20cm high = obstacle
        this->get_parameter("obstacle_height_threshold", obstacle_height_threshold_);
        this->declare_parameter<double>("history_duration", 2.0);  // 2초간 높이맵 유지
        this->get_parameter("history_duration", history_duration_);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/downsampled_cloud", 10, 
            std::bind(&HeightMapPlannerNode::pointCloudCallback, this, std::placeholders::_1));

        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector_heightmap", 10);
        heightmap_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/height_map_markers", 10);
        risk_map_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/risk_map_markers", 10);
        
        last_time_ = this->now();
        ground_height_ = 0.0f;  // Will be estimated from initial frames
        ground_height_calibrated_ = false;
        
        RCLCPP_INFO(this->get_logger(), "HeightMap 2.5D Planner Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Grid resolution: %.2f m, History duration: %.1f s", 
                   grid_resolution_, history_duration_);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Front view filtering
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        if (front_view_only_) {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(0.0, max_range_);
            pass.filter(*cloud_filtered);
        } else {
            *cloud_filtered = *cloud;
        }

        // Ground height calibration (first few frames)
        if (!ground_height_calibrated_) {
            calibrateGroundHeight(cloud_filtered);
            return;  // Skip processing until calibrated
        }

        // Build current height map
        std::unordered_map<std::string, HeightCell> current_height_map;
        buildHeightMap(cloud_filtered, current_height_map, current_time);
        
        // Add to history and clean old data
        updateHeightMapHistory(current_height_map, current_time);
        
        // Build accumulated height map from history
        std::unordered_map<std::string, HeightCell> accumulated_height_map;
        buildAccumulatedHeightMap(accumulated_height_map, current_time);
        
        // Analyze traversability and detect negative obstacles
        analyzeTraversability(accumulated_height_map);
        
        // Plan path using height-aware ray casting
        double safe_angle = planPathWithHeightAwareness(accumulated_height_map);
        
        // Apply temporal smoothing
        const double max_angular_velocity = M_PI / 2.0; // 90 deg/s
        double angle_diff = angles::shortest_angular_distance(smoothed_angle_, safe_angle);
        double max_angle_change = max_angular_velocity * dt;
        double angle_change = std::clamp(angle_diff, -max_angle_change, max_angle_change);
        smoothed_angle_ += angle_change;
        smoothed_angle_ = angles::normalize_angle(smoothed_angle_);

        // Publish results
        publishSafeVector(msg->header, smoothed_angle_);
        publishHeightMapVisualization(msg->header, accumulated_height_map);
    }
    
    void calibrateGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        if (cloud->points.empty()) return;
        
        // Collect Z values from points close to sensor (presumably ground)
        std::vector<float> ground_candidates;
        for (const auto& point : cloud->points) {
            if (point.x > 0.5 && point.x < 2.0 && 
                std::abs(point.y) < 1.0 && 
                point.z > -1.0 && point.z < 1.0) {
                ground_candidates.push_back(point.z);
            }
        }
        
        if (ground_candidates.size() > 10) {
            // Use median as robust ground height estimate
            std::sort(ground_candidates.begin(), ground_candidates.end());
            ground_height_ = ground_candidates[ground_candidates.size() / 2];
            ground_height_calibrated_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Ground height calibrated to: %.3f m", ground_height_);
        }
    }
    
    void buildHeightMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                       std::unordered_map<std::string, HeightCell>& height_map, 
                       rclcpp::Time timestamp)
    {
        height_map.clear();
        
        for (const auto& point : cloud->points) {
            // Convert to grid coordinates
            int grid_x = static_cast<int>(std::floor(point.x / grid_resolution_));
            int grid_y = static_cast<int>(std::floor(point.y / grid_resolution_));
            
            // Create grid key
            std::string key = std::to_string(grid_x) + "," + std::to_string(grid_y);
            
            // Add point to height cell
            height_map[key].addPoint(point.z, timestamp);
        }
    }
    
    void updateHeightMapHistory(const std::unordered_map<std::string, HeightCell>& current_map, 
                               rclcpp::Time current_time)
    {
        // Add current map to history
        HeightMapFrame frame;
        frame.timestamp = current_time;
        frame.cells = current_map;
        height_history_.push_back(frame);
        
        // Remove old frames beyond history duration
        double history_cutoff = (current_time - rclcpp::Duration::from_seconds(history_duration_)).seconds();
        
        while (!height_history_.empty() && 
               height_history_.front().timestamp.seconds() < history_cutoff) {
            height_history_.pop_front();
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Height history size: %zu frames", height_history_.size());
    }
    
    void buildAccumulatedHeightMap(std::unordered_map<std::string, HeightCell>& accumulated_map,
                                  rclcpp::Time current_time)
    {
        accumulated_map.clear();
        
        // Merge all frames in history
        for (const auto& frame : height_history_) {
            for (const auto& [key, cell] : frame.cells) {
                if (accumulated_map.find(key) == accumulated_map.end()) {
                    // New cell
                    accumulated_map[key] = cell;
                } else {
                    // Merge with existing cell
                    auto& existing_cell = accumulated_map[key];
                    existing_cell.min_z = std::min(existing_cell.min_z, cell.min_z);
                    existing_cell.max_z = std::max(existing_cell.max_z, cell.max_z);
                    
                    // Weighted average based on point count
                    float total_points = existing_cell.point_count + cell.point_count;
                    existing_cell.mean_z = (existing_cell.mean_z * existing_cell.point_count + 
                                          cell.mean_z * cell.point_count) / total_points;
                    existing_cell.point_count = total_points;
                    
                    // Keep the most recent timestamp
                    if (cell.last_seen > existing_cell.last_seen) {
                        existing_cell.last_seen = cell.last_seen;
                    }
                }
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Accumulated height map: %zu cells", accumulated_map.size());
    }
    
    void analyzeTraversability(std::unordered_map<std::string, HeightCell>& height_map)
    {
        for (auto& [key, cell] : height_map) {
            if (cell.point_count == 0) continue;
            
            // Calculate drop risk based on height relative to ground
            float height_diff_from_ground = cell.min_z - ground_height_;
            
            // Negative obstacles (drops, holes)
            if (height_diff_from_ground < -drop_threshold_) {
                cell.drop_risk = std::min(1.0f, static_cast<float>(std::abs(height_diff_from_ground) / drop_threshold_));
                cell.is_traversable = false;
            }
            // Positive obstacles (walls, furniture)
            else if (cell.max_z - ground_height_ > obstacle_height_threshold_) {
                cell.drop_risk = 0.5f;  // Medium risk
                cell.is_traversable = false;
            }
            // Rough terrain
            else if (cell.getHeightRange() > ground_height_tolerance_) {
                cell.drop_risk = 0.3f;  // Low risk but bumpy
                cell.is_traversable = true;  // Still traversable but penalized
            }
            else {
                cell.drop_risk = 0.0f;
                cell.is_traversable = true;
            }
        }
    }
    
    double planPathWithHeightAwareness(const std::unordered_map<std::string, HeightCell>& height_map)
    {
        const int num_rays = 15;
        const double ray_clearance = 0.15;  // 15cm clearance on each side of ray
        std::vector<double> scores(num_rays, 0.0);
        
        for (int i = 0; i < num_rays; ++i) {
            double angle = (i * (M_PI / (num_rays - 1))) - (M_PI / 2.0);
            Eigen::Vector2f ray_dir(cos(angle), sin(angle));
            
            double total_clearance = max_range_;
            double total_risk_penalty = 0.0;
            int cells_checked = 0;
            
            // Cast ray and check grid cells along the path
            for (double dist = 0.2; dist < max_range_; dist += grid_resolution_) {
                Eigen::Vector2f ray_point = ray_dir * dist;
                
                // Check cells around the ray (for clearance)
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
            
            // Calculate score: prioritize clearance, heavily penalize risk
            double avg_risk = cells_checked > 0 ? total_risk_penalty / cells_checked : 0.0;
            scores[i] = total_clearance * (1.0 - avg_risk * 2.0);  // Risk penalty is doubled
            
            // Neighbor smoothing (from original algorithm)
            if (i > 0 && i < num_rays - 1) {
                double neighbor_boost = 0.1 * (scores[i-1] + scores[i+1]);
                scores[i] += neighbor_boost;
            }
        }
        
        // Find best ray (highest score)
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
            
            // Parse grid coordinates from key
            size_t comma_pos = key.find(',');
            int grid_x = std::stoi(key.substr(0, comma_pos));
            int grid_y = std::stoi(key.substr(comma_pos + 1));
            
            // 그리드 중심 위치 계산 (올바른 방식)
            double world_x = (grid_x + 0.5) * grid_resolution_;
            double world_y = (grid_y + 0.5) * grid_resolution_;
            
            // 격자 셀을 큐브로 표시 (진짜 HeightMap 시각화)
            visualization_msgs::msg::Marker grid_marker;
            grid_marker.header = header;
            grid_marker.ns = "height_grid";
            grid_marker.id = marker_id++;
            grid_marker.type = visualization_msgs::msg::Marker::CUBE;
            grid_marker.action = visualization_msgs::msg::Marker::ADD;
            grid_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
            
            // 위치: 그리드 중심, 높이는 지면 기준
            grid_marker.pose.position.x = world_x;
            grid_marker.pose.position.y = world_y;
            grid_marker.pose.position.z = ground_height_ + (cell.max_z - cell.min_z) / 2.0;
            grid_marker.pose.orientation.w = 1.0;
            
            // 크기: 그리드 해상도에 맞춤
            grid_marker.scale.x = grid_resolution_ * 0.9;  // 90% 크기로 격자 간격 시각화
            grid_marker.scale.y = grid_resolution_ * 0.9;
            
            // 높이 차이에 따른 큐브 높이 설정
            double height_range = cell.max_z - cell.min_z;
            if (height_range < 0.01) height_range = 0.01;  // 최소 1cm
            grid_marker.scale.z = height_range;
            
            // 색상: 통과 가능성과 높이 기반
            if (!cell.is_traversable) {
                if (cell.drop_risk > 0.5) {
                    // 높은 위험 - 진한 빨간색
                    grid_marker.color.r = 0.9;
                    grid_marker.color.g = 0.0;
                    grid_marker.color.b = 0.0;
                } else {
                    // 일반 장애물 - 주황색
                    grid_marker.color.r = 1.0;
                    grid_marker.color.g = 0.4;
                    grid_marker.color.b = 0.0;
                }
            } else {
                // 통과 가능 - 높이에 따른 녹색 그라데이션
                double height_offset = cell.mean_z - ground_height_;
                if (std::abs(height_offset) < 0.05) {
                    // 지면 레벨 - 밝은 녹색
                    grid_marker.color.r = 0.0;
                    grid_marker.color.g = 0.8;
                    grid_marker.color.b = 0.2;
                } else {
                    // 약간 높거나 낮음 - 연한 녹색
                    grid_marker.color.r = 0.2;
                    grid_marker.color.g = 0.6;
                    grid_marker.color.b = 0.3;
                }
            }
            grid_marker.color.a = 0.7;
            
            markers.markers.push_back(grid_marker);
        }
        
        height_publisher_->publish(markers);
    }
                } else {
                    // Orange cylinders for obstacles  
                    risk_marker.color.r = 1.0;
                    risk_marker.color.g = 0.5;
                    risk_marker.color.b = 0.0;
                }
                risk_marker.color.a = cell.drop_risk;  // 위험도에 따른 투명도
                
                risk_markers.markers.push_back(risk_marker);
            }
            
            marker_id++;
        }
        
        heightmap_publisher_->publish(height_markers);
        risk_map_publisher_->publish(risk_markers);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr heightmap_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_map_publisher_;
    
    bool front_view_only_;
    double grid_resolution_;
    double max_range_;
    double ground_height_tolerance_;
    double drop_threshold_;
    double obstacle_height_threshold_;
    double history_duration_;  // 높이맵 히스토리 유지 시간
    
    double smoothed_angle_;
    rclcpp::Time last_time_;
    
    float ground_height_;
    bool ground_height_calibrated_;
    
    // 시간 누적을 위한 높이맵 히스토리
    std::deque<HeightMapFrame> height_history_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeightMapPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
