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
    
    GapInfo(int start, int end, double width, double depth, double center) 
        : start_sector(start), end_sector(end), width_degrees(width), 
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
        visualization_msgs::msg::MarkerArray sector_markers;
        visualization_msgs::msg::MarkerArray gap_markers;
        
        double sector_angle_width = M_PI / num_sectors_;
        
        // 섹터별 장애물 거리 시각화 (화살표)
        for (int i = 0; i < num_sectors_; ++i) {
            visualization_msgs::msg::Marker arrow;
            arrow.header = header;
            arrow.ns = "ftg_sectors";
            arrow.id = i;
            arrow.type = visualization_msgs::msg::Marker::ARROW;
            arrow.action = visualization_msgs::msg::Marker::ADD;
            arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            double sector_angle = (i * sector_angle_width) - M_PI/2.0;
            double distance = std::min(sector_distances[i], max_range_);
            
            arrow.points.resize(2);
            arrow.points[0].x = 0.0;
            arrow.points[0].y = 0.0;
            arrow.points[0].z = 0.05;
            arrow.points[1].x = distance * cos(sector_angle);
            arrow.points[1].y = distance * sin(sector_angle);
            arrow.points[1].z = 0.05;
            
            // 화살표 크기
            arrow.scale.x = 0.02;  // 샤프트 두께
            arrow.scale.y = 0.04;  // 헤드 두께  
            arrow.scale.z = 0.04;  // 헤드 길이
            
            // 거리에 따른 색상
            if (distance > max_range_ * 0.9) {
                arrow.color.r = 0.0; arrow.color.g = 0.8; arrow.color.b = 0.0;  // 진한 녹색
            } else if (distance > max_range_ * 0.6) {
                arrow.color.r = 0.3; arrow.color.g = 1.0; arrow.color.b = 0.0;  // 연한 녹색
            } else if (distance > max_range_ * 0.3) {
                arrow.color.r = 1.0; arrow.color.g = 0.6; arrow.color.b = 0.0;  // 주황
            } else {
                arrow.color.r = 1.0; arrow.color.g = 0.0; arrow.color.b = 0.0;  // 빨강
            }
            arrow.color.a = 0.8;
            
            sector_markers.markers.push_back(arrow);
        }
        
        // 갭 시각화 - 부채꼴 모양으로 실제 갭 영역 표시
        for (size_t i = 0; i < gaps.size(); ++i) {
            const auto& gap = gaps[i];
            
            // 갭의 시작과 끝 각도 계산
            double start_angle = (gap.start_sector * sector_angle_width) - M_PI/2.0;
            double end_angle = ((gap.end_sector + 1) * sector_angle_width) - M_PI/2.0;
            
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
            
            gap_arc.scale.x = 0.02;  // 라인 두께
            
            // 점수에 따른 색상
            double normalized_score = std::min(gap.score / 25.0, 1.0);
            gap_arc.color.r = 0.0;
            gap_arc.color.g = 0.5 + normalized_score * 0.5;  // 0.5~1.0
            gap_arc.color.b = 1.0 - normalized_score * 0.3;  // 1.0~0.7
            gap_arc.color.a = 0.8;
            
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
            
            direction_arrow.scale.x = 0.03;  // 두꺼운 샤프트
            direction_arrow.scale.y = 0.06;  // 큰 헤드
            direction_arrow.scale.z = 0.08;
            
            // 최고 점수 갭은 특별히 표시
            if (i == 0) {  // 첫 번째 갭이 최고 점수라고 가정
                direction_arrow.color.r = 1.0;
                direction_arrow.color.g = 1.0;
                direction_arrow.color.b = 0.0;  // 노란색
            } else {
                direction_arrow.color.r = 0.0;
                direction_arrow.color.g = 1.0;
                direction_arrow.color.b = 0.0;  // 녹색
            }
            direction_arrow.color.a = 0.9;
            
            gap_markers.markers.push_back(direction_arrow);
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
