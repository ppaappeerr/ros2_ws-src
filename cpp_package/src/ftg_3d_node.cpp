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

struct GapInfo {
    int start_sector;
    int end_sector;
    double width_degrees;
    double depth_meters;
    double center_angle;
    double score;

    GapInfo() : start_sector(0), end_sector(0), width_degrees(0), depth_meters(0), center_angle(0), score(0) {} // Default constructor
    
    GapInfo(int start, int end, double width, double depth, double center) 
        : start_sector(start), end_sector(end), width_degrees(width), 
          depth_meters(depth), center_angle(center), score(0.0) {}
};

class FollowTheGap3DNode : public rclcpp::Node
{
public:
    FollowTheGap3DNode() : Node("follow_the_gap_3d_node"), smoothed_angle_(M_PI) // facing -X
    {
        // Parameters
        this->declare_parameter<bool>("front_view_only", true);
        this->get_parameter("front_view_only", front_view_only_);
        this->declare_parameter<double>("corridor_width", 0.4);  // 40cm corridor width
        this->get_parameter("corridor_width", corridor_width_);
        this->declare_parameter<double>("max_range", 5.0);
        this->get_parameter("max_range", max_range_);
        this->declare_parameter<double>("min_gap_width", 5.0);  // 5 degrees minimum (much more sensitive)
        this->get_parameter("min_gap_width", min_gap_width_);
        this->declare_parameter<double>("safety_margin", 0.1);  // Extra safety margin
        this->get_parameter("safety_margin", safety_margin_);
        this->declare_parameter<int>("num_sectors", 36);  // Number of angular sectors (10 degree resolution)
        this->get_parameter("num_sectors", num_sectors_);
        this->declare_parameter<bool>("show_sectors", false);
        this->get_parameter("show_sectors", show_sectors_);

        // New parameters for stability and visualization
        this->declare_parameter<int>("min_points_per_sector", 3);
        this->get_parameter("min_points_per_sector", min_points_per_sector_);
        this->declare_parameter<bool>("compact_viz", true); // Show only the best gap
        this->get_parameter("compact_viz", compact_viz_);
        this->declare_parameter<double>("gap_inertia", 0.3); // 0: no inertia, 1: sticky
        this->get_parameter("gap_inertia", gap_inertia_);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/downsampled_cloud", 10, 
            std::bind(&FollowTheGap3DNode::pointCloudCallback, this, std::placeholders::_1));

        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector", 10);
        // NEW: dedicated gap topic
        gap_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ftg_gaps", 10);
        // Debug (optional sectors etc.)
        debug_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_debug", 10);
        // sector_publisher_ removed; sectors go to debug if enabled
        
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Follow-the-Gap 3D Node started (front is -X).");
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
            pass.setFilterLimits(-max_range_, 0.0);
            pass.filter(*cloud_filtered);
        } else {
            *cloud_filtered = *cloud;
        }

        // Build distance profile for each angular sector
        std::vector<double> sector_distances = buildSectorDistanceProfile(cloud_filtered);
        
        // Find continuous gaps
        std::vector<GapInfo> gaps = findContinuousGaps(sector_distances);
        
        // Score and select the best gap
        double safe_angle = selectBestGap(gaps);
        
        // Apply temporal smoothing
        const double max_angular_velocity = M_PI / 2.0; // 90 deg/s
        double angle_diff = angles::shortest_angular_distance(smoothed_angle_, safe_angle);
        double max_angle_change = max_angular_velocity * dt;
        double angle_change = std::clamp(angle_diff, -max_angle_change, max_angle_change);
        smoothed_angle_ = angles::normalize_angle(smoothed_angle_ + angle_change);

        // Publish results
        publishSafeVector(msg->header, smoothed_angle_);
        publishGapVisualization(msg->header, gaps, sector_distances);
    }
    
    std::vector<double> buildSectorDistanceProfile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<double> sector_distances(num_sectors_, max_range_);
        std::vector<int> sector_counts(num_sectors_, 0); // Keep track of points per sector
        double sector_angle_width = M_PI / num_sectors_; // 180 deg FOV centered at -X
        
        for (const auto& point : cloud->points) {
            if (point.x >= 0) continue; // skip behind new forward
            
            double point_angle = std::atan2(point.y, point.x); // angle in original frame
            
            // Shift so that -X (pi) maps to 0 (center of new FOV) -> subtract pi, then add pi/2 for index in [0,pi]
            double shifted = angles::normalize_angle(point_angle - M_PI); // now around 0
            double normalized_angle = shifted + M_PI/2.0; // [ -pi/2, +pi/2 ] -> shift to [0,pi]
            if (normalized_angle < 0 || normalized_angle > M_PI) continue; // outside FOV
            
            int sector_idx = static_cast<int>(normalized_angle / sector_angle_width);
            sector_idx = std::max(0, std::min(num_sectors_-1, sector_idx));
            
            double sector_center_angle = (sector_idx * sector_angle_width) - M_PI/2.0; // [-pi/2,pi/2] relative to -X center
            double world_angle = sector_center_angle + M_PI; // convert back to world frame
            
            Eigen::Vector2f sector_dir(cos(world_angle), sin(world_angle));
            Eigen::Vector2f point_2d(point.x, point.y);
            
            // Calculate perpendicular distance to sector center line
            double dist_to_sector_line = std::abs(point.x * sector_dir.y() - point.y * sector_dir.x());
            
            if (dist_to_sector_line < (corridor_width_/2.0 + safety_margin_)) {
                // Point is within the corridor for this sector
                double point_distance = point_2d.norm();
                sector_distances[sector_idx] = std::min(sector_distances[sector_idx], point_distance);
                sector_counts[sector_idx]++;
            }
        }

        // Penalize sectors with too few points (treat as unknown/unsafe)
        for(int i = 0; i < num_sectors_; ++i) {
            if (sector_counts[i] < min_points_per_sector_) {
                sector_distances[i] = 0.1; // Treat as a close obstacle
            }
        }
        
        return sector_distances;
    }
    
    std::vector<GapInfo> findContinuousGaps(const std::vector<double>& sector_distances)
    {
        std::vector<GapInfo> gaps;
        
        // Adaptive gap threshold based on environment
        // If most sectors are blocked, lower the threshold to find narrow passages
        int free_sectors = 0;
        for (double dist : sector_distances) {
            if (dist > max_range_ * 0.6) free_sectors++;
        }
        
        // Dynamic threshold: if environment is crowded, be more lenient
        double gap_threshold;
        if (free_sectors < num_sectors_ * 0.3) {
            // Crowded environment (like maze): look for any passage > corridor width
            gap_threshold = corridor_width_ * 1.5;  // 60cm for 40cm corridor
        } else {
            // Open environment: use stricter threshold
            gap_threshold = max_range_ * 0.8;
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Gap threshold: %.2f m (free_sectors: %d/%d)", 
                    gap_threshold, free_sectors, num_sectors_);
        
        int gap_start = -1;
        
        for (int i = 0; i < num_sectors_; ++i) {
            bool is_free = sector_distances[i] > gap_threshold;
            
            if (is_free && gap_start == -1) {
                // Start of a new gap
                gap_start = i;
            } else if (!is_free && gap_start != -1) {
                // End of current gap
                createGapFromIndices(gaps, gap_start, i - 1, sector_distances);
                gap_start = -1;
            }
        }
        
        // Handle gap that extends to the end
        if (gap_start != -1) {
            createGapFromIndices(gaps, gap_start, num_sectors_ - 1, sector_distances);
        }
        
        // Adaptive minimum width filtering
        double min_width_threshold = min_gap_width_;
        if (free_sectors < num_sectors_ * 0.2) {
            // Very crowded: accept even smaller gaps
            min_width_threshold = min_gap_width_ * 0.5;  // 2.5 degrees
        }
        
        gaps.erase(std::remove_if(gaps.begin(), gaps.end(), 
            [min_width_threshold](const GapInfo& gap) { 
                return gap.width_degrees < min_width_threshold; 
            }), gaps.end());
        
        RCLCPP_DEBUG(this->get_logger(), "Found %zu gaps with min_width=%.1f°", 
                    gaps.size(), min_width_threshold);
        
        return gaps;
    }
    
    void createGapFromIndices(std::vector<GapInfo>& gaps, int start_idx, int end_idx, 
                             const std::vector<double>& sector_distances)
    {
        double sector_angle_width = M_PI / num_sectors_;
        
        // Calculate gap properties
        double start_angle_rel = (start_idx * sector_angle_width) - M_PI/2.0;
        double end_angle_rel = ((end_idx + 1) * sector_angle_width) - M_PI/2.0;
        double center_rel = (start_angle_rel + end_angle_rel)/2.0;
        double start_angle_world = start_angle_rel + M_PI;
        double end_angle_world = end_angle_rel + M_PI;
        double center_angle_world = center_rel + M_PI;
        double width_degrees = (end_angle_world - start_angle_world) * 180.0 / M_PI;
        
        // Calculate average depth in the gap
        double total_depth=0.0;
        for(int i=start_idx;i<=end_idx;++i) total_depth += sector_distances[i];
        double avg_depth = total_depth / (end_idx - start_idx + 1);
        
        gaps.emplace_back(start_idx, end_idx, width_degrees, avg_depth, center_angle_world);
    }
    
    double selectBestGap(std::vector<GapInfo>& gaps)
    {
        if (gaps.empty()) {
            // No gaps found, go straight
            RCLCPP_WARN(this->get_logger(), "No gaps found, maintaining previous angle.");
            return smoothed_angle_; // Return last known good angle instead of snapping to M_PI
        }
        
        // Score each gap
        for (auto& gap : gaps) {
            // Base score: depth * width
            gap.score = gap.depth_meters * gap.width_degrees;
            
            // Bonus for central gaps (prefer going straight)
            double angle_from_center = std::abs(angles::shortest_angular_distance(M_PI, gap.center_angle));
            double centrality_bonus = (M_PI/2.0 - angle_from_center) / (M_PI/2.0);  // 0 to 1
            gap.score += centrality_bonus * 2.0;  // Up to 2.0 bonus points
            
            // Bonus for wider gaps
            double width_bonus = std::min(gap.width_degrees / 60.0, 1.0);  // Cap at 60 degrees
            gap.score += width_bonus * 1.0;

            // Inertia: add bonus to gaps close to the previously smoothed angle
            double inertia_diff = std::abs(angles::shortest_angular_distance(smoothed_angle_, gap.center_angle));
            double inertia_bonus = (M_PI - inertia_diff) / M_PI; // 1 if same, 0 if opposite
            gap.score *= (1.0 + gap_inertia_ * inertia_bonus);
        }
        
        // Find the highest-scoring gap
        auto best_gap_it = std::max_element(gaps.begin(), gaps.end(),
            [](const GapInfo& a, const GapInfo& b) { return a.score < b.score; });
        
        RCLCPP_DEBUG(this->get_logger(), "Selected gap: angle=%.1f°, width=%.1f°, depth=%.2fm, score=%.2f",
                    best_gap_it->center_angle * 180.0 / M_PI, best_gap_it->width_degrees, 
                    best_gap_it->depth_meters, best_gap_it->score);
        
        // Store the best gap for compact visualization
        if (!gaps.empty()) {
            best_gap_ = *best_gap_it;
        }

        return best_gap_it->center_angle;
    }
    
    void publishSafeVector(const std_msgs::msg::Header& header, double angle)
    {
        geometry_msgs::msg::Vector3Stamped msg_out;
        msg_out.header = header;
        msg_out.vector.x = cos(angle);
        msg_out.vector.y = sin(angle);
        msg_out.vector.z = 0;
        vector_publisher_->publish(msg_out);
    }
    
    void publishGapVisualization(const std_msgs::msg::Header& header, 
                                const std::vector<GapInfo>& gaps,
                                const std::vector<double>& sector_distances)
    {
        visualization_msgs::msg::MarkerArray sector_markers; // only if show_sectors_
        visualization_msgs::msg::MarkerArray gap_markers;
        
        double sector_angle_width = M_PI / num_sectors_;
        if (show_sectors_) {
            for (int i = 0; i < num_sectors_; ++i) {
                visualization_msgs::msg::Marker arrow;
                arrow.header = header;
                arrow.ns = "ftg_sectors";
                arrow.id = i;
                arrow.type = visualization_msgs::msg::Marker::ARROW;
                arrow.action = visualization_msgs::msg::Marker::ADD;
                arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
                
                double sector_rel = (i * sector_angle_width) - M_PI/2.0;
                double world_angle = sector_rel + M_PI;
                double distance = std::min(sector_distances[i], max_range_);
                
                arrow.points.resize(2);
                arrow.points[0].x = 0.0; arrow.points[0].y = 0.0; arrow.points[0].z = 0.05;
                arrow.points[1].x = distance * cos(world_angle);
                arrow.points[1].y = distance * sin(world_angle);
                arrow.points[1].z = 0.05;
                
                // 화살표 크기
                arrow.scale.x = 0.01; // thinner
                arrow.scale.y = 0.02;
                arrow.scale.z = 0.02;
                
                // desaturate for less clutter
                arrow.color.r = 0.3; arrow.color.g = 0.3; arrow.color.b = 0.3; arrow.color.a = 0.15;
                
                sector_markers.markers.push_back(arrow);
            }
        }
        // Gap arcs + direction arrows
        for (size_t i = 0; i < gaps.size(); ++i) {
            const auto& gap = gaps[i];
            
            // 갭의 시작과 끝 각도 계산
            double start_angle= ((gap.start_sector * sector_angle_width) - M_PI/2.0) + M_PI;
            double end_angle = (((gap.end_sector + 1) * sector_angle_width) - M_PI/2.0) + M_PI;
            
            // 갭 영역을 부채꼴로 시각화 (라인 스트립 사용)
            visualization_msgs::msg::Marker gap_arc;
            gap_arc.header = header;
            gap_arc.ns = "ftg_gaps";
            gap_arc.id = i;
            gap_arc.type = visualization_msgs::msg::Marker::LINE_STRIP;
            gap_arc.action = visualization_msgs::msg::Marker::ADD;
            gap_arc.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            // 부채꼴 그리기
            gap_arc.points.clear();
            
            // 중심점
            geometry_msgs::msg::Point center;
            center.x = 0.0; center.y = 0.0; center.z = 0.1;
            gap_arc.points.push_back(center);
            
            // 갭 영역의 호 그리기
            int arc_points = std::max(5, static_cast<int>(gap.width_degrees / 2.0));  // 2도마다 점
            for (int j = 0; j <= arc_points; ++j) {
                double angle = start_angle + (end_angle - start_angle) * j / arc_points;
                geometry_msgs::msg::Point point;
                point.x = gap.depth_meters * 0.8 * cos(angle);
                point.y = gap.depth_meters * 0.8 * sin(angle);
                point.z = 0.1;
                gap_arc.points.push_back(point);
            }
            
            // 중심점으로 돌아가기
            gap_arc.points.push_back(center);
            
            gap_arc.scale.x = 0.025;
            
            // 점수에 따른 색상
            double normalized_score = std::min(gap.score / 25.0, 1.0);
            gap_arc.color.r = 0.0;
            gap_arc.color.g = 0.6 + 0.4 * normalized_score;
            gap_arc.color.b = 1.0 - 0.3 * normalized_score;
            gap_arc.color.a = 0.85;
            
            gap_markers.markers.push_back(gap_arc);
            
            // 갭 중심 방향 화살표
            visualization_msgs::msg::Marker direction_arrow;
            direction_arrow.header = header;
            direction_arrow.ns = "ftg_gaps";
            direction_arrow.id = i + 1000;  // ID 겹침 방지
            direction_arrow.type = visualization_msgs::msg::Marker::ARROW;
            direction_arrow.action = visualization_msgs::msg::Marker::ADD;
            direction_arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            direction_arrow.points.resize(2);
            direction_arrow.points[0].x = 0.0;
            direction_arrow.points[0].y = 0.0;
            direction_arrow.points[0].z = 0.15;
            direction_arrow.points[1].x = gap.depth_meters * 0.7 * cos(gap.center_angle);
            direction_arrow.points[1].y = gap.depth_meters * 0.7 * sin(gap.center_angle);
            direction_arrow.points[1].z = 0.15;
            
            direction_arrow.scale.x = 0.035; direction_arrow.scale.y = 0.07; direction_arrow.scale.z = 0.09;
            if (i==0){ direction_arrow.color.r=1.0; direction_arrow.color.g=1.0; direction_arrow.color.b=0.0; }
            else { direction_arrow.color.r=0.0; direction_arrow.color.g=1.0; direction_arrow.color.b=0.0; }
            direction_arrow.color.a = 0.95;
            
            gap_markers.markers.push_back(direction_arrow);
        }
        // Publish
        if (show_sectors_ && !sector_markers.markers.empty()) debug_publisher_->publish(sector_markers);
        if (!gap_markers.markers.empty()) gap_publisher_->publish(gap_markers);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_publisher_;
    
    bool front_view_only_;
    double corridor_width_;
    double max_range_;
    double min_gap_width_;
    double safety_margin_;
    int num_sectors_;
    bool show_sectors_;
    int min_points_per_sector_;
    bool compact_viz_;
    double gap_inertia_;
    GapInfo best_gap_;
    
    double smoothed_angle_;
    rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowTheGap3DNode>());
    rclcpp::shutdown();
    return 0;
}
