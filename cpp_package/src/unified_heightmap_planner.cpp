#include "base_path_planner.hpp"
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

/**
 * @brief P5: HeightMap 2.5D 방식 경로 계획기 (통일된 구조)
 * 
 * 핵심 로직:
 * - 3D 포인트를 격자로 분할하여 높이맵 생성
 * - DROP/OBSTACLE/UNEVEN 지형 분류
 * - 시간적 누적 및 지수 감쇠 적용
 */
class UnifiedHeightMapPlanner : public BasePathPlanner
{
public:
    UnifiedHeightMapPlanner() : BasePathPlanner("unified_heightmap_planner", "/downsampled_cloud"),
                               ground_height_(0.0f), ground_height_calibrated_(false)
    {
        // P5 고유 파라미터
        this->declare_parameter<double>("grid_resolution", 0.05);          // 5cm 격자
        this->declare_parameter<double>("ground_height_tolerance", 0.1);   // 지면 높이 허용 오차
        this->declare_parameter<double>("drop_threshold", 0.15);           // 드롭 감지 임계값
        this->declare_parameter<double>("obstacle_height_threshold", 0.2); // 장애물 높이 임계값
        this->declare_parameter<double>("history_duration", 2.0);          // point_cloud_sweeper와 맞춤
        this->declare_parameter<double>("decay_tau", 0.8);                 // 지수 감쇠 시상수
        this->declare_parameter<int>("num_rays", 50);                      // 방향 해상도
        this->declare_parameter<double>("ray_clearance", 0.2);             // Ray 폭
        
        this->get_parameter("grid_resolution", grid_resolution_);
        this->get_parameter("ground_height_tolerance", ground_height_tolerance_);
        this->get_parameter("drop_threshold", drop_threshold_);
        this->get_parameter("obstacle_height_threshold", obstacle_height_threshold_);
        this->get_parameter("history_duration", history_duration_);
        this->get_parameter("decay_tau", decay_tau_);
        this->get_parameter("num_rays", num_rays_);
        this->get_parameter("ray_clearance", ray_clearance_);
        
        // 커스텀 시각화를 위한 추가 퍼블리셔
        pillar_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/height_pillars", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "P5 플래너 초기화: %.2fm 격자, %.1fs 이력, %d rays", 
            grid_resolution_, history_duration_, num_rays_);
    }

protected:
    double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override
    {
        rclcpp::Time current_time = this->now();
        
        // 1. 지면 높이 캘리브레이션
        if (!ground_height_calibrated_) {
            calibrateGroundHeight(cloud);
            if (!ground_height_calibrated_) {
                return M_PI;  // 캘리브레이션 중이면 전방 유지
            }
        }
        
        // 2. 현재 프레임 높이맵 구성
        std::unordered_map<std::string, HeightCell> current_map;
        buildHeightMap(cloud, current_map, current_time);
        
        // 3. 시간적 이력 업데이트
        updateHeightMapHistory(current_map, current_time);
        
        // 4. 지수 감쇠 적용한 누적 높이맵 구성
        std::unordered_map<std::string, HeightCell> accumulated_map;
        buildAccumulatedHeightMapWithDecay(accumulated_map, current_time);
        
        // 5. 이동 가능성 분석
        analyzeTraversability(accumulated_map);
        
        // 6. 경로 계획
        double ideal_angle = planPathWithHeightAwareness(accumulated_map);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "P5: 격자 %zu개, 지면=%.2fm, 각도=%.1f°", 
            accumulated_map.size(), ground_height_, ideal_angle * 180/M_PI);
        
        return ideal_angle;
    }
    
    void publishCustomVisualization(const std_msgs::msg::Header& header,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& /* cloud */,
                                   double direction) override
    {
        // HeightMap 전용 pillar 시각화
        publishPillarVisualization(header, direction);
    }

private:
    void calibrateGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<float> ground_candidates;
        
        for (const auto& point : *cloud) {
            // 전방 0.5~2.0m 범위에서 지면 후보 수집
            if (point.x < -0.5 && point.x > -2.0) {
                ground_candidates.push_back(point.z);
            }
        }
        
        if (ground_candidates.size() > 50) {
            std::sort(ground_candidates.begin(), ground_candidates.end());
            ground_height_ = ground_candidates[ground_candidates.size() / 2];  // 중앙값
            ground_height_calibrated_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                "지면 높이 캘리브레이션: %.3fm (샘플 %zu개)", 
                ground_height_, ground_candidates.size());
        }
    }
    
    void buildHeightMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       std::unordered_map<std::string, HeightCell>& height_map,
                       rclcpp::Time timestamp)
    {
        height_map.clear();
        
        for (const auto& point : *cloud) {
            int grid_x = static_cast<int>(std::floor(point.x / grid_resolution_));
            int grid_y = static_cast<int>(std::floor(point.y / grid_resolution_));
            std::string key = std::to_string(grid_x) + "," + std::to_string(grid_y);
            
            height_map[key].addPoint(point.z, timestamp);
        }
    }
    
    void updateHeightMapHistory(const std::unordered_map<std::string, HeightCell>& current_map,
                               rclcpp::Time current_time)
    {
        // 새 프레임 추가
        HeightMapFrame frame;
        frame.timestamp = current_time;
        frame.cells = current_map;
        height_history_.push_back(frame);
        
        // 오래된 프레임 제거
        double history_cutoff = (current_time - rclcpp::Duration::from_seconds(history_duration_)).seconds();
        while (!height_history_.empty() && 
               height_history_.front().timestamp.seconds() < history_cutoff) {
            height_history_.pop_front();
        }
    }
    
    void buildAccumulatedHeightMapWithDecay(std::unordered_map<std::string, HeightCell>& accumulated_map,
                                           rclcpp::Time current_time)
    {
        accumulated_map.clear();
        
        for (const auto& frame : height_history_) {
            double age = (current_time - frame.timestamp).seconds();
            double time_weight = std::exp(-age / decay_tau_);
            
            for (const auto& [key, cell] : frame.cells) {
                if (accumulated_map.find(key) == accumulated_map.end()) {
                    HeightCell weighted_cell = cell;
                    weighted_cell.point_count = static_cast<int>(cell.point_count * time_weight);
                    accumulated_map[key] = weighted_cell;
                } else {
                    auto& existing_cell = accumulated_map[key];
                    existing_cell.min_z = std::min(existing_cell.min_z, cell.min_z);
                    existing_cell.max_z = std::max(existing_cell.max_z, cell.max_z);
                    
                    float weighted_points = cell.point_count * time_weight;
                    float total_points = existing_cell.point_count + weighted_points;
                    if (total_points > 0) {
                        existing_cell.mean_z = (existing_cell.mean_z * existing_cell.point_count + 
                                              cell.mean_z * weighted_points) / total_points;
                    }
                    existing_cell.point_count += weighted_points;
                    
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
                // 드롭 위험
                cell.is_traversable = false;
                cell.drop_risk = std::min(1.0f, static_cast<float>(std::abs(height_diff_from_ground) / drop_threshold_));
            } else if (cell.max_z - ground_height_ > obstacle_height_threshold_) {
                // 장애물
                cell.is_traversable = false;
                cell.drop_risk = 0.0f;
            } else if (cell.getHeightRange() > ground_height_tolerance_) {
                // 불균등 지형
                cell.is_traversable = false;
                cell.drop_risk = 0.2f;
            } else {
                // 이동 가능
                cell.is_traversable = true;
                cell.drop_risk = 0.0f;
            }
        }
    }
    
    double planPathWithHeightAwareness(const std::unordered_map<std::string, HeightCell>& height_map)
    {
        std::vector<double> raw_scores(num_rays_, 0.0);
        std::vector<int> evidences(num_rays_, 0);
        
        for (int i = 0; i < num_rays_; ++i) {
            double rel_angle = (i * (10.0 * M_PI / 9.0) / (num_rays_ - 1)) - (5.0 * M_PI / 9.0);  // +-100도 범위
            double ray_angle = rel_angle + M_PI;  // -X 전방 기준
            
            Eigen::Vector2f ray_dir(cos(ray_angle), sin(ray_angle));
            double total_clearance = roi_max_radius_;
            double total_risk_penalty = 0.0;
            int cells_checked = 0;
            int traversable_count = 0;
            
            // Ray를 따라 스캔
            for (double dist = 0.2; dist < roi_max_radius_; dist += grid_resolution_) {
                Eigen::Vector2f ray_point = ray_dir * dist;
                
                for (double offset = -ray_clearance_; offset <= ray_clearance_; offset += grid_resolution_) {
                    Eigen::Vector2f perp_dir(-ray_dir.y(), ray_dir.x());
                    Eigen::Vector2f check_point = ray_point + perp_dir * offset;
                    
                    if (check_point.x() > 0) continue;  // 후방 제외
                    
                    int grid_x = static_cast<int>(std::floor(check_point.x() / grid_resolution_));
                    int grid_y = static_cast<int>(std::floor(check_point.y() / grid_resolution_));
                    std::string key = std::to_string(grid_x) + "," + std::to_string(grid_y);
                    
                    auto it = height_map.find(key);
                    if (it != height_map.end()) {
                        const HeightCell& cell = it->second;
                        cells_checked++;
                        
                        if (!cell.is_traversable) {
                            total_clearance = std::min(total_clearance, dist);
                        } else {
                            traversable_count++;
                        }
                        
                        total_risk_penalty += cell.drop_risk;
                    }
                }
            }
            
            // 점수 계산
            double avg_risk = (cells_checked > 0) ? total_risk_penalty / cells_checked : 0.0;
            raw_scores[i] = total_clearance * (1.0 - avg_risk * 2.0);
            evidences[i] = traversable_count;
            
            // 증거 없는 방향 페널티
            if (traversable_count == 0) {
                raw_scores[i] *= 0.3;
            }
        }
        
        // Evidence-based scoring 적용
        std::vector<double> final_scores;
        applyEvidenceScoring(raw_scores, evidences, final_scores);
        
        // 최적 방향 선택
        int best_ray_idx = std::distance(final_scores.begin(), 
                                        std::max_element(final_scores.begin(), final_scores.end()));
        
        double ideal_angle = ((best_ray_idx * M_PI / (num_rays_ - 1)) - (M_PI / 2.0)) + M_PI;
        return ideal_angle;
    }
    
    void publishPillarVisualization(const std_msgs::msg::Header& header, double /* direction */)
    {
        // 실제 pillar 시각화 - accumulated height map 기반
        visualization_msgs::msg::MarkerArray pillar_array;
        int marker_id = 0;
        
        // 최근 계산된 height map이 있는 경우에만 시각화
        if (height_history_.empty()) {
            // keepalive 마커만 발행
            visualization_msgs::msg::Marker keepalive;
            keepalive.header = header;
            keepalive.ns = "height_pillars";
            keepalive.id = 999999;
            keepalive.type = visualization_msgs::msg::Marker::SPHERE;
            keepalive.action = visualization_msgs::msg::Marker::ADD;
            keepalive.scale.x = 0.001;
            keepalive.scale.y = 0.001;
            keepalive.scale.z = 0.001;
            keepalive.color.a = 0.0;
            keepalive.lifetime = rclcpp::Duration::from_seconds(0.2);
            pillar_array.markers.push_back(keepalive);
        } else {
            // 최근 프레임의 height map 사용
            const auto& latest_frame = height_history_.back();
            
            for (const auto& [key, cell] : latest_frame.cells) {
                if (cell.point_count == 0) continue;
                
                // 모든 셀을 표시 (디버깅을 위해 필터링 완전 제거)
                // bool show_safe_cells = (marker_id % 10 == 0);  // 10개 중 1개만 표시
                // if (cell.is_traversable && cell.drop_risk < 0.1f && !show_safe_cells) continue;
                
                // 격자 좌표 파싱
                size_t comma_pos = key.find(',');
                if (comma_pos == std::string::npos) continue;
                
                int grid_x = std::stoi(key.substr(0, comma_pos));
                int grid_y = std::stoi(key.substr(comma_pos + 1));
                
                // 월드 좌표 계산
                double world_x = (grid_x + 0.5) * grid_resolution_;
                double world_y = (grid_y + 0.5) * grid_resolution_;
                
                // 전방 뷰 범위를 더 넓게 (디버깅용)
                double distance = std::sqrt(world_x*world_x + world_y*world_y);
                if (world_x > 1.0 || distance > roi_max_radius_ + 1.0) {
                    continue;  // 더 관대한 범위
                }
                
                // 위험도별 pillar 생성
                visualization_msgs::msg::Marker pillar;
                pillar.header = header;
                pillar.ns = "height_pillars";
                pillar.id = marker_id++;
                pillar.type = visualization_msgs::msg::Marker::CYLINDER;
                pillar.action = visualization_msgs::msg::Marker::ADD;
                pillar.lifetime = rclcpp::Duration::from_seconds(0.3);
                
                // 위치 설정
                pillar.pose.position.x = world_x;
                pillar.pose.position.y = world_y;
                
                // 위험 유형별 높이 및 색상 설정
                double pillar_height = 0.15;
                float height_diff_from_ground = cell.min_z - ground_height_;
                
                if (height_diff_from_ground < -drop_threshold_) {
                    // 드롭 위험 (빨간색)
                    double drop_depth = std::min(0.5, static_cast<double>(std::abs(height_diff_from_ground)));
                    pillar_height = std::max(0.1, drop_depth * 0.6);
                    pillar.color.r = 1.0;
                    pillar.color.g = 0.2;
                    pillar.color.b = 0.0;
                    pillar.color.a = 0.8;
                } else if (cell.max_z - ground_height_ > obstacle_height_threshold_) {
                    // 장애물 (주황색)
                    double obstacle_height = std::min(0.5, static_cast<double>(cell.max_z - ground_height_));
                    pillar_height = std::max(0.1, obstacle_height * 0.7);
                    pillar.color.r = 1.0;
                    pillar.color.g = 0.5;
                    pillar.color.b = 0.0;
                    pillar.color.a = 0.8;
                } else if (!cell.is_traversable) {
                    // 불균등 지형 (노란색)
                    pillar_height = 0.12;
                    pillar.color.r = 1.0;
                    pillar.color.g = 1.0;
                    pillar.color.b = 0.2;
                    pillar.color.a = 0.7;
                } else {
                    // 일반적인 지형 (파란색, 더 눈에 띄게)
                    pillar_height = 0.12;
                    pillar.color.r = 0.2;
                    pillar.color.g = 0.6;
                    pillar.color.b = 1.0;
                    pillar.color.a = 0.8;
                }
                
                // pillar 크기 설정
                pillar.pose.position.z = ground_height_ + pillar_height / 2.0;
                pillar.pose.orientation.w = 1.0;
                
                double radius = std::max(0.04, grid_resolution_ * 0.8);  // 더 큰 반지름
                pillar.scale.x = radius;
                pillar.scale.y = radius;
                pillar.scale.z = pillar_height;
                
                pillar_array.markers.push_back(pillar);
                
                // 더 많은 마커 허용 (디버깅용)
                if (marker_id > 1000) break;
            }
            
            // keepalive 마커도 추가
            if (pillar_array.markers.empty()) {
                visualization_msgs::msg::Marker keepalive;
                keepalive.header = header;
                keepalive.ns = "height_pillars";
                keepalive.id = 999999;
                keepalive.type = visualization_msgs::msg::Marker::SPHERE;
                keepalive.action = visualization_msgs::msg::Marker::ADD;
                keepalive.scale.x = 0.001;
                keepalive.scale.y = 0.001;
                keepalive.scale.z = 0.001;
                keepalive.color.a = 0.0;
                keepalive.lifetime = rclcpp::Duration::from_seconds(0.2);
                pillar_array.markers.push_back(keepalive);
            }
        }
        
        pillar_publisher_->publish(pillar_array);
        
        // 디버깅용 로그 (항상 출력)
        RCLCPP_INFO(this->get_logger(), "HeightMap pillar 시각화: %zu개 마커 발행", pillar_array.markers.size());
        if (!height_history_.empty()) {
            RCLCPP_INFO(this->get_logger(), "최신 프레임에 %zu개 셀 존재", height_history_.back().cells.size());
        }
    }

private:
    // P5 고유 파라미터
    double grid_resolution_;
    double ground_height_tolerance_;
    double drop_threshold_;
    double obstacle_height_threshold_;
    double history_duration_;
    double decay_tau_;
    int num_rays_;
    double ray_clearance_;
    
    // 상태 변수
    float ground_height_;
    bool ground_height_calibrated_;
    std::deque<HeightMapFrame> height_history_;
    
    // 추가 퍼블리셔
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pillar_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedHeightMapPlanner>());
    rclcpp::shutdown();
    return 0;
}