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
    HeightMapPlannerNode() : Node("heightmap_planner_node"), smoothed_angle_(M_PI)
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
        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector", 10);
        // NEW: risk & direction debug
        risk_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_debug", 10);
        // NEW: pillars (drop risk) separate topic
        pillar_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/height_pillars", 10);
        
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
            pass.setFilterLimits(-max_range_, 0.0);
            pass.filter(*cloud);
        }
        
        if (!ground_height_calibrated_) {
            calibrateGroundHeight(cloud);
            RCLCPP_INFO(this->get_logger(), "Ground height calibration: %.3f m", ground_height_);
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
        
        RCLCPP_DEBUG(this->get_logger(), "Generated %zu grid cells from %zu cloud points", 
                    accumulated_map.size(), cloud->points.size());
        
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
        publishRiskMapVisualization(msg->header, accumulated_map, smoothed_angle_);
        
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
        std::vector<int> traversable_counts(num_rays,0);
        for (int i = 0; i < num_rays; ++i) {
            double rel_angle = (i * (M_PI / (num_rays - 1))) - (M_PI / 2.0); // [-pi/2,pi/2] around +X
            double ray_angle = rel_angle + M_PI; // shift to -X forward
            Eigen::Vector2f ray_dir(cos(ray_angle), sin(ray_angle));
            double total_clearance = max_range_;
            double total_risk_penalty = 0.0; int cells_checked = 0; int traversable_local = 0;
            double ray_clearance = 0.2;
            for (double dist = 0.2; dist < max_range_; dist += grid_resolution_) {
                Eigen::Vector2f ray_point = ray_dir * dist;
                for (double offset = -ray_clearance; offset <= ray_clearance; offset += grid_resolution_) {
                    Eigen::Vector2f perp_dir(-ray_dir.y(), ray_dir.x());
                    Eigen::Vector2f check_point = ray_point + perp_dir * offset;
                    // Discard points that fall outside FOV wedge (avoid wrapping artifacts)
                    if (check_point.x() > 0) continue; // behind new forward reference
                    int grid_x = static_cast<int>(std::floor(check_point.x() / grid_resolution_));
                    int grid_y = static_cast<int>(std::floor(check_point.y() / grid_resolution_));
                    std::string key = std::to_string(grid_x) + "," + std::to_string(grid_y);
                    auto it = height_map.find(key);
                    if (it != height_map.end()) {
                        const HeightCell & cell = it->second;
                        if (!cell.is_traversable) {
                            total_clearance = std::min(total_clearance, dist);
                        } else {
                            traversable_local++;
                        }
                        total_risk_penalty += cell.drop_risk;
                        cells_checked++;
                    }
                }
            }
            traversable_counts[i] = traversable_local;
            double avg_risk = cells_checked > 0 ? total_risk_penalty / cells_checked : 0.0;
            scores[i] = total_clearance * (1.0 - avg_risk * 2.0);
        }
        // Penalize rays with zero traversable cells (edge pointing into void)
        for (int i=0;i<num_rays;++i){ if (traversable_counts[i]==0) scores[i] *= 0.3; }
        // Central bias to avoid drifting toward FOV edges in sparse maps
        double center_idx = (num_rays - 1)/2.0;
        for (int i=0;i<num_rays;++i){ double norm_pos = std::abs(i-center_idx)/center_idx; scores[i] *= (1.0 - 0.25*norm_pos*norm_pos); }
        int best_ray_idx = std::distance(scores.begin(), std::max_element(scores.begin(), scores.end()));
        double ideal_angle = ((best_ray_idx * (M_PI / (num_rays - 1))) - (M_PI / 2.0)) + M_PI;
        
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
    
    void publishRiskMapVisualization(const std_msgs::msg::Header& header,
                                    const std::unordered_map<std::string, HeightCell>& height_map,
                                    double safe_angle)
    {
        visualization_msgs::msg::MarkerArray debug_markers; // safe direction
        visualization_msgs::msg::MarkerArray pillar_markers; // refined risk pillars
        int id_debug = 0;
        int id_pillar = 0;
        // Safe direction arrow (unchanged)
        visualization_msgs::msg::Marker direction_arrow;
        direction_arrow.header = header;
        direction_arrow.ns = "safe_direction";
        direction_arrow.id = id_debug++;
        direction_arrow.type = visualization_msgs::msg::Marker::ARROW;
        direction_arrow.action = visualization_msgs::msg::Marker::ADD;
        direction_arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
        direction_arrow.points.resize(2);
        direction_arrow.points[0].x=0; direction_arrow.points[0].y=0; direction_arrow.points[0].z=0.2;
        direction_arrow.points[1].x=2.0*cos(safe_angle); direction_arrow.points[1].y=2.0*sin(safe_angle); direction_arrow.points[1].z=0.2;
        direction_arrow.scale.x=0.05; direction_arrow.scale.y=0.1; direction_arrow.scale.z=0.15;
        direction_arrow.color.r=0.0; direction_arrow.color.g=1.0; direction_arrow.color.b=0.0; direction_arrow.color.a=0.9;
        debug_markers.markers.push_back(direction_arrow);

        // Pillars (refined)
        for (const auto& [key, cell] : height_map) {
            if (cell.point_count == 0) continue;
            // Skip very low risk
            if (cell.drop_risk < 0.05 && cell.is_traversable) continue;

            size_t comma_pos = key.find(',');
            int grid_x = std::stoi(key.substr(0, comma_pos));
            int grid_y = std::stoi(key.substr(comma_pos + 1));
            double world_x = (grid_x + 0.5) * grid_resolution_;
            double world_y = (grid_y + 0.5) * grid_resolution_;

            double min_offset = cell.min_z - ground_height_;   // negative if drop
            double max_offset = cell.max_z - ground_height_;   // positive if obstacle
            double range = cell.getHeightRange();

            double pillar_height = 0.0;  // visual scale.z
            double severity = 0.0;       // 0~1 for color mapping
            enum class PillarType { DROP, OBSTACLE, UNEVEN, SAFE } type = PillarType::SAFE;

            if (min_offset < -drop_threshold_) {
                // Drop: height proportional to depth below ground
                double drop_depth = std::min(0.6, std::abs(min_offset));
                pillar_height = std::max(0.12, drop_depth * 0.8);
                severity = std::min(1.0, std::abs(min_offset) / (drop_threshold_ * 3.0));
                type = PillarType::DROP;
            } else if (max_offset > obstacle_height_threshold_) {
                // Obstacle protruding up
                double protrude = std::min(0.6, max_offset);
                pillar_height = std::max(0.10, protrude * 0.9);
                severity = std::min(1.0, max_offset / (obstacle_height_threshold_ * 3.0));
                type = PillarType::OBSTACLE;
            } else if (!cell.is_traversable || range > ground_height_tolerance_) {
                // Uneven terrain / roughness
                double rough = std::min(0.4, range * 2.0);
                pillar_height = std::max(0.08, rough);
                severity = std::min(1.0, range / (ground_height_tolerance_ * 2.0));
                type = PillarType::UNEVEN;
            } else {
                continue; // traversable & low risk: skip
            }

            visualization_msgs::msg::Marker pillar;
            pillar.header = header;
            pillar.ns = "height_pillars";
            pillar.id = id_pillar++;
            pillar.type = visualization_msgs::msg::Marker::CYLINDER;
            pillar.action = visualization_msgs::msg::Marker::ADD;
            pillar.lifetime = rclcpp::Duration::from_seconds(0.3);
            pillar.pose.position.x = world_x;
            pillar.pose.position.y = world_y;
            // Base sits slightly above ground for visibility
            pillar.pose.position.z = ground_height_ + pillar_height / 2.0;
            pillar.pose.orientation.w = 1.0;

            // Thin radius relative to grid resolution; widen slightly with severity
            double base_radius = std::max(0.25 * grid_resolution_, 0.01);
            double radius = base_radius * (1.0 + 0.6 * severity);
            pillar.scale.x = radius;
            pillar.scale.y = radius;
            pillar.scale.z = pillar_height;

            // Color scheme per type
            if (type == PillarType::DROP) {
                // Yellow -> Red
                pillar.color.r = 1.0;
                pillar.color.g = 1.0 - 0.8 * severity;
                pillar.color.b = 0.0;
            } else if (type == PillarType::OBSTACLE) {
                // Orange -> Deep Red
                pillar.color.r = 1.0;
                pillar.color.g = 0.5 - 0.4 * severity;
                pillar.color.b = 0.0;
            } else { // UNEVEN
                // Greenish -> Orange (roughness)
                pillar.color.r = 0.3 + 0.5 * severity;
                pillar.color.g = 0.8 - 0.5 * severity;
                pillar.color.b = 0.1;
            }
            pillar.color.a = 0.88;

            pillar_markers.markers.push_back(pillar);
        }

        if (!debug_markers.markers.empty()) risk_publisher_->publish(debug_markers);
        if (!pillar_markers.markers.empty()) pillar_publisher_->publish(pillar_markers);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risk_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pillar_publisher_;
    
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
