#include "base_path_planner.hpp"
#include "angles/angles.h"

struct GapInfo {
    int start_sector;
    int end_sector;
    double width_degrees;
    double depth_meters;
    double center_angle;
    double score;
    
    GapInfo() : start_sector(0), end_sector(0), width_degrees(0), depth_meters(0), center_angle(0), score(0) {}
    
    GapInfo(int start, int end, double width, double depth, double center) 
        : start_sector(start), end_sector(end), width_degrees(width), 
          depth_meters(depth), center_angle(center), score(0.0) {}
};

/**
 * @brief P4: Follow-the-Gap 3D 방식 경로 계획기 (통일된 구조)
 * 
 * 핵심 로직:
 * - 360도를 36개 섹터로 분할 (10도 해상도)
 * - 각 섹터의 거리 프로파일 생성
 * - 연속된 빈 섹터를 "갭"으로 인식
 * - 가장 넓고 깊은 갭으로 진행
 */
class UnifiedFollowTheGap3D : public BasePathPlanner
{
public:
    UnifiedFollowTheGap3D() : BasePathPlanner("unified_follow_the_gap_3d", "/downsampled_cloud")
    {
        // P4 고유 파라미터
        this->declare_parameter<int>("num_sectors", 36);              // 36개 섹터 (10도 해상도)
        this->declare_parameter<double>("corridor_width", 0.4);       // 40cm corridor
        this->declare_parameter<double>("max_detection_range", 5.0);  // 최대 탐지 거리
        this->declare_parameter<double>("min_gap_width", 15.0);       // 최소 갭 폭 (도)
        this->declare_parameter<double>("safety_margin", 0.1);        // 안전 여백
        this->declare_parameter<int>("min_points_per_sector", 3);     // 섹터당 최소 포인트
        
        this->get_parameter("num_sectors", num_sectors_);
        this->get_parameter("corridor_width", corridor_width_);
        this->get_parameter("max_detection_range", max_detection_range_);
        this->get_parameter("min_gap_width", min_gap_width_);
        this->get_parameter("safety_margin", safety_margin_);
        this->get_parameter("min_points_per_sector", min_points_per_sector_);
        
        // 커스텀 시각화를 위한 추가 퍼블리셔
        gap_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ftg_gaps", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "P4 플래너 초기화: %d 섹터, %.2fm corridor, %.1f° 최소갭", 
            num_sectors_, corridor_width_, min_gap_width_);
    }

protected:
    double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override
    {
        // 1. 섹터별 거리 프로파일 구성
        std::vector<double> sector_distances(num_sectors_, max_detection_range_);
        std::vector<int> sector_counts(num_sectors_, 0);  // 각 섹터의 포인트 수
        
        buildSectorDistanceProfile(cloud, sector_distances, sector_counts);
        
        // 2. 부족한 증거를 가진 섹터 페널티 적용
        applySectorEvidencePenalty(sector_distances, sector_counts);
        
        // 3. 갭 탐지
        std::vector<GapInfo> gaps = detectGaps(sector_distances);
        
        // 4. 갭 스코어링 및 선택
        GapInfo best_gap = selectBestGap(gaps);
        
        // 5. 시각화를 위해 최근 갭들과 섹터 거리들 저장
        latest_gaps_ = gaps;
        latest_sector_distances_ = sector_distances;
        
        // 5. 각도 계산
        double ideal_angle;
        if (best_gap.width_degrees > 0) {
            // 전방 뷰(-X 기준)로 변환
            ideal_angle = best_gap.center_angle + M_PI;
        } else {
            // 갭이 없으면 전방 직진
            ideal_angle = M_PI;
        }
        
        RCLCPP_DEBUG(this->get_logger(), 
            "P4: %zu개 갭, 최적갭 %.1f°(폭=%.1f°, 깊이=%.2fm)", 
            gaps.size(), best_gap.center_angle * 180/M_PI, 
            best_gap.width_degrees, best_gap.depth_meters);
        
        return ideal_angle;
    }
    
    void publishCustomVisualization(const std_msgs::msg::Header& header,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& /* cloud */,
                                   double direction) override
    {
        // FTG 전용 갭 시각화
        publishGapVisualization(header, direction);
    }

private:
    void buildSectorDistanceProfile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   std::vector<double>& sector_distances,
                                   std::vector<int>& sector_counts)
    {
        std::fill(sector_distances.begin(), sector_distances.end(), max_detection_range_);
        std::fill(sector_counts.begin(), sector_counts.end(), 0);
        
        for (const auto& point : *cloud) {
            // 전방 뷰만 처리 (-X 기준)
            if (point.x > 0) continue;  // 후방 제외
            
            // 포인트의 각도 계산 (atan2 결과: -π ~ +π, +X 기준)
            double angle = atan2(point.y, point.x);
            double distance = sqrt(point.x * point.x + point.y * point.y);
            
            // 전방 뷰 범위로 제한 (+-90도, -X 중심)
            // -X는 180도, 따라서 90~270도 범위만 처리
            double angle_degrees = angle * 180.0 / M_PI;
            if (angle_degrees < 0) angle_degrees += 360.0;
            
            // 전방 시야각 제한 (90도 ~ 270도)
            if (angle_degrees < 90.0 || angle_degrees > 270.0) continue;
            
            int sector = static_cast<int>(angle_degrees / (360.0 / num_sectors_)) % num_sectors_;
            
            // 해당 섹터의 최단 거리 업데이트
            if (distance < sector_distances[sector]) {
                sector_distances[sector] = distance;
            }
            sector_counts[sector]++;
        }
    }
    
    void applySectorEvidencePenalty(std::vector<double>& sector_distances,
                                   const std::vector<int>& sector_counts)
    {
        for (int i = 0; i < num_sectors_; ++i) {
            if (sector_counts[i] < min_points_per_sector_) {
                // 증거 부족한 섹터는 차단된 것으로 간주
                sector_distances[i] = 0.8;  // 80cm로 제한
            }
        }
    }
    
    std::vector<GapInfo> detectGaps(const std::vector<double>& sector_distances)
    {
        std::vector<GapInfo> gaps;
        double degrees_per_sector = 360.0 / num_sectors_;
        double min_gap_sectors = min_gap_width_ / degrees_per_sector;
        
        bool in_gap = false;
        int gap_start = -1;
        
        // 전방 뷰 섹터만 고려 (대략 270도 ~ 90도, +X 기준)
        int front_start = static_cast<int>((270.0 / degrees_per_sector));  // 270도부터
        // int front_end = static_cast<int>((90.0 / degrees_per_sector));     // 90도까지 (사용안함)
        
        for (int i = 0; i < num_sectors_; ++i) {
            int sector_idx = (front_start + i) % num_sectors_;
            
            // 전방 뷰 범위를 벗어나면 중단
            if (i > num_sectors_ / 2) break;
            
            bool is_free = sector_distances[sector_idx] > (corridor_width_ + safety_margin_);
            
            if (is_free && !in_gap) {
                // 갭 시작
                in_gap = true;
                gap_start = sector_idx;
            } else if (!is_free && in_gap) {
                // 갭 종료
                in_gap = false;
                int gap_end = (sector_idx - 1 + num_sectors_) % num_sectors_;
                
                // 갭 폭 계산
                int gap_width_sectors = calculateGapWidth(gap_start, gap_end);
                if (gap_width_sectors >= static_cast<int>(min_gap_sectors)) {
                    GapInfo gap = createGapInfo(gap_start, gap_end, sector_distances, degrees_per_sector);
                    gaps.push_back(gap);
                }
            }
        }
        
        // 루프 끝에서 갭이 계속되고 있으면 마감
        if (in_gap) {
            int gap_end = (front_start + num_sectors_ / 2 - 1) % num_sectors_;
            int gap_width_sectors = calculateGapWidth(gap_start, gap_end);
            if (gap_width_sectors >= static_cast<int>(min_gap_sectors)) {
                GapInfo gap = createGapInfo(gap_start, gap_end, sector_distances, degrees_per_sector);
                gaps.push_back(gap);
            }
        }
        
        return gaps;
    }
    
    int calculateGapWidth(int start, int end) const
    {
        if (end >= start) {
            return end - start + 1;
        } else {
            return (num_sectors_ - start) + end + 1;
        }
    }
    
    GapInfo createGapInfo(int start, int end, const std::vector<double>& distances, double degrees_per_sector)
    {
        GapInfo gap;
        gap.start_sector = start;
        gap.end_sector = end;
        gap.width_degrees = calculateGapWidth(start, end) * degrees_per_sector;
        
        // 갭 중심 각도 계산
        int center_sector = start;
        if (end >= start) {
            center_sector = (start + end) / 2;
        } else {
            center_sector = ((start + end + num_sectors_) / 2) % num_sectors_;
        }
        
        gap.center_angle = (center_sector * degrees_per_sector) * M_PI / 180.0;  // 라디안 변환
        if (gap.center_angle > M_PI) gap.center_angle -= 2 * M_PI;  // [-π, π] 범위
        
        // 갭 깊이 = 갭 내 최대 거리
        gap.depth_meters = 0;
        for (int s = start; s != (end + 1) % num_sectors_; s = (s + 1) % num_sectors_) {
            gap.depth_meters = std::max(gap.depth_meters, distances[s]);
            if (s == end) break;  // 순환 방지
        }
        
        return gap;
    }
    
    GapInfo selectBestGap(const std::vector<GapInfo>& gaps)
    {
        if (gaps.empty()) {
            return GapInfo();  // 빈 갭
        }
        
        GapInfo best_gap = gaps[0];
        double best_score = -1;
        
        for (const auto& gap : gaps) {
            // 갭 점수 = 폭 × 깊이 (정규화)
            double width_score = std::min(gap.width_degrees / 90.0, 1.0);    // 최대 90도
            double depth_score = std::min(gap.depth_meters / max_detection_range_, 1.0);
            double combined_score = 0.6 * width_score + 0.4 * depth_score;
            
            // 전방 선호도 (중앙에 가까울수록 높은 점수)
            double front_preference = 1.0 - std::abs(gap.center_angle) / M_PI;
            combined_score *= (0.8 + 0.2 * front_preference);
            
            if (combined_score > best_score) {
                best_score = combined_score;
                best_gap = gap;
                best_gap.score = combined_score;
            }
        }
        
        return best_gap;
    }
    
    void publishGapVisualization(const std_msgs::msg::Header& header, double direction)
    {
        visualization_msgs::msg::MarkerArray gap_array;
        
        if (latest_gaps_.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "갭이 없어서 시각화 건너뜀");
            return;
        }
        
        double degrees_per_sector = 360.0 / num_sectors_;
        
        // 실제 계산된 갭들을 시각화
        for (size_t i = 0; i < latest_gaps_.size(); ++i) {
            const auto& gap = latest_gaps_[i];
            
            // 갭의 시작과 끝 각도 계산
            double start_angle_rel = (gap.start_sector * degrees_per_sector) - 180.0;  // -180~180도 범위
            double end_angle_rel = ((gap.end_sector + 1) * degrees_per_sector) - 180.0;
            
            double start_angle_world = (start_angle_rel * M_PI / 180.0) + M_PI;  // 월드 좌표계로 변환
            double end_angle_world = (end_angle_rel * M_PI / 180.0) + M_PI;
            
            // 갭 영역을 부채꼴로 시각화
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
                double angle = start_angle_world + (end_angle_world - start_angle_world) * j / arc_points;
                geometry_msgs::msg::Point point;
                point.x = gap.depth_meters * 0.8 * cos(angle);
                point.y = gap.depth_meters * 0.8 * sin(angle);
                point.z = 0.1;
                gap_arc.points.push_back(point);
            }
            
            // 중심점으로 돌아가기
            gap_arc.points.push_back(center);
            
            gap_arc.scale.x = 0.025;
            
            // 점수에 따른 색상 (최고 점수는 노란색, 나머지는 초록색)
            double max_score = 0.0;
            for (const auto& g : latest_gaps_) {
                max_score = std::max(max_score, g.score);
            }
            
            if (gap.score >= max_score - 0.1) {  // 최고 점수 갭
                gap_arc.color.r = 1.0; gap_arc.color.g = 1.0; gap_arc.color.b = 0.0;
            } else {
                gap_arc.color.r = 0.0; gap_arc.color.g = 0.8; gap_arc.color.b = 0.2;
            }
            gap_arc.color.a = 0.7;
            
            gap_array.markers.push_back(gap_arc);
            
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
            
            direction_arrow.scale.x = 0.035; 
            direction_arrow.scale.y = 0.07; 
            direction_arrow.scale.z = 0.09;
            
            // 색상 (최고 점수는 노란색, 나머지는 초록색)
            if (gap.score >= max_score - 0.1) {
                direction_arrow.color.r = 1.0; direction_arrow.color.g = 1.0; direction_arrow.color.b = 0.0;
            } else {
                direction_arrow.color.r = 0.0; direction_arrow.color.g = 1.0; direction_arrow.color.b = 0.0;
            }
            direction_arrow.color.a = 0.95;
            
            gap_array.markers.push_back(direction_arrow);
        }
        
        gap_publisher_->publish(gap_array);
        
        RCLCPP_INFO(this->get_logger(), "FTG 갭 시각화: %zu개 실제 갭 표시", latest_gaps_.size());
    }

private:
    // P4 고유 파라미터
    int num_sectors_;
    double corridor_width_;
    double max_detection_range_;
    double min_gap_width_;
    double safety_margin_;
    int min_points_per_sector_;
    
    // 추가 퍼블리셔
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gap_publisher_;
    
    // 최근 계산된 갭들을 저장 (시각화용)
    std::vector<GapInfo> latest_gaps_;
    std::vector<double> latest_sector_distances_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedFollowTheGap3D>());
    rclcpp::shutdown();
    return 0;
}