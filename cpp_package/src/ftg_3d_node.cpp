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
    int start_idx;
    int end_idx;
    double width_degrees;
    double depth_meters;
    double center_angle;
    double score;
    
    GapInfo(int start, int end, double width, double depth, double center) 
        : start_idx(start), end_idx(end), width_degrees(width), 
          depth_meters(depth), center_angle(center), score(0.0) {}
};

class FollowTheGap3DNode : public rclcpp::Node
{
public:
    FollowTheGap3DNode() : Node("follow_the_gap_3d_node"), smoothed_angle_(0.0)
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
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/downsampled_cloud", 10, 
            std::bind(&FollowTheGap3DNode::pointCloudCallback, this, std::placeholders::_1));

        vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector_ftg3d", 10);
        gap_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/gap_markers", 10);
        sector_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/sector_markers", 10);
        
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Follow-the-Gap 3D Node has been started.");
        RCLCPP_INFO(this->get_logger(), "Corridor width: %.2f m, Sectors: %d", corridor_width_, num_sectors_);
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
        smoothed_angle_ += angle_change;
        smoothed_angle_ = angles::normalize_angle(smoothed_angle_);

        // Publish results
        publishSafeVector(msg->header, smoothed_angle_);
        publishGapVisualization(msg->header, gaps, sector_distances);
    }
    
    std::vector<double> buildSectorDistanceProfile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<double> sector_distances(num_sectors_, max_range_);
        
        // Angular resolution per sector
        double sector_angle_width = M_PI / num_sectors_;  // Full 180-degree front view
        
        for (const auto& point : cloud->points) {
            if (point.x <= 0) continue;  // Skip points behind sensor
            
            // Calculate angle of this point
            double point_angle = std::atan2(point.y, point.x);
            
            // Convert to sector index (0 = -90°, num_sectors-1 = +90°)
            double normalized_angle = point_angle + M_PI/2.0;  // Shift to [0, π]
            int sector_idx = static_cast<int>(normalized_angle / sector_angle_width);
            
            // Clamp to valid range
            sector_idx = std::max(0, std::min(num_sectors_ - 1, sector_idx));
            
            // Check if point is within the 3D corridor for this sector
            double sector_center_angle = (sector_idx * sector_angle_width) - M_PI/2.0;
            Eigen::Vector2f sector_dir(cos(sector_center_angle), sin(sector_center_angle));
            Eigen::Vector2f point_2d(point.x, point.y);
            
            // Calculate perpendicular distance to sector center line
            double dist_to_sector_line = std::abs(point.x * sector_dir.y() - point.y * sector_dir.x());
            
            if (dist_to_sector_line < (corridor_width_ / 2.0 + safety_margin_)) {
                // Point is within the corridor for this sector
                double point_distance = point_2d.norm();
                sector_distances[sector_idx] = std::min(sector_distances[sector_idx], point_distance);
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
        double start_angle = (start_idx * sector_angle_width) - M_PI/2.0;
        double end_angle = ((end_idx + 1) * sector_angle_width) - M_PI/2.0;
        double center_angle = (start_angle + end_angle) / 2.0;
        double width_degrees = (end_angle - start_angle) * 180.0 / M_PI;
        
        // Calculate average depth in the gap
        double total_depth = 0.0;
        for (int i = start_idx; i <= end_idx; ++i) {
            total_depth += sector_distances[i];
        }
        double avg_depth = total_depth / (end_idx - start_idx + 1);
        
        gaps.emplace_back(start_idx, end_idx, width_degrees, avg_depth, center_angle);
    }
    
    double selectBestGap(std::vector<GapInfo>& gaps)
    {
        if (gaps.empty()) {
            // No gaps found, go straight
            RCLCPP_WARN(this->get_logger(), "No gaps found, going straight");
            return 0.0;
        }
        
        // Score each gap
        for (auto& gap : gaps) {
            // Base score: depth * width
            gap.score = gap.depth_meters * gap.width_degrees;
            
            // Bonus for central gaps (prefer going straight)
            double angle_from_center = std::abs(gap.center_angle);
            double centrality_bonus = (M_PI/2.0 - angle_from_center) / (M_PI/2.0);  // 0 to 1
            gap.score += centrality_bonus * 2.0;  // Up to 2.0 bonus points
            
            // Bonus for wider gaps
            double width_bonus = std::min(gap.width_degrees / 60.0, 1.0);  // Cap at 60 degrees
            gap.score += width_bonus * 1.0;
        }
        
        // Find the highest-scoring gap
        auto best_gap = std::max_element(gaps.begin(), gaps.end(),
            [](const GapInfo& a, const GapInfo& b) { return a.score < b.score; });
        
        RCLCPP_DEBUG(this->get_logger(), "Selected gap: angle=%.1f°, width=%.1f°, depth=%.2fm, score=%.2f",
                    best_gap->center_angle * 180.0 / M_PI, best_gap->width_degrees, 
                    best_gap->depth_meters, best_gap->score);
        
        return best_gap->center_angle;
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
    
    void publishGapVisualization(const std_msgs::msg::Header& header, 
                                const std::vector<GapInfo>& gaps,
                                const std::vector<double>& sector_distances)
    {
        // Visualize sectors
        visualization_msgs::msg::MarkerArray sector_markers;
        visualization_msgs::msg::MarkerArray gap_markers;
        
        // Clear previous markers first
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        sector_markers.markers.push_back(clear_marker);
        gap_markers.markers.push_back(clear_marker);
        
        double sector_angle_width = M_PI / num_sectors_;
        
        for (int i = 0; i < num_sectors_; ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "sectors";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            double sector_angle = (i * sector_angle_width) - M_PI/2.0;
            double distance = sector_distances[i];
            
            marker.points.resize(2);
            marker.points[0].x = 0.0;
            marker.points[0].y = 0.0;
            marker.points[0].z = 0.0;
            marker.points[1].x = distance * cos(sector_angle);
            marker.points[1].y = distance * sin(sector_angle);
            marker.points[1].z = 0.0;
            
            marker.scale.x = 0.01;  // 더 얇은 화살표
            marker.scale.y = 0.02;  // 작은 헤드
            marker.scale.z = 0.02;  // 작은 헤드
            
            // Color based on distance
            if (distance > max_range_ * 0.8) {
                // Green for free space
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else if (distance > max_range_ * 0.4) {
                // Yellow for medium distance
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            } else {
                // Red for close obstacles
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            }
            marker.color.a = 0.6;
            
            sector_markers.markers.push_back(marker);
        }
        
        // Visualize gaps
        for (size_t i = 0; i < gaps.size(); ++i) {
            const auto& gap = gaps[i];
            
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "gaps";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.pose.position.x = (gap.depth_meters * 0.5) * cos(gap.center_angle);
            marker.pose.position.y = (gap.depth_meters * 0.5) * sin(gap.center_angle);
            marker.pose.position.z = 0.1;
            marker.pose.orientation.w = 1.0;
            
            marker.scale.x = 0.3;  // 더 작은 크기
            marker.scale.y = 0.3;
            marker.scale.z = 0.05 + (gap.score / 50.0);  // 점수에 따른 높이
            
            // Color intensity based on score
            double normalized_score = std::min(gap.score / 20.0, 1.0);
            marker.color.r = 0.0;
            marker.color.g = normalized_score;
            marker.color.b = 1.0 - normalized_score;
            marker.color.a = 0.7;
            
            gap_markers.markers.push_back(marker);
        }
        
        sector_publisher_->publish(sector_markers);
        gap_publisher_->publish(gap_markers);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sector_publisher_;
    
    bool front_view_only_;
    double corridor_width_;
    double max_range_;
    double min_gap_width_;
    double safety_margin_;
    int num_sectors_;
    
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
