#include "base_path_planner.hpp"

/**
 * @brief P2: 순수 2D 스캔 누적 방식 경로 계획기 (통일된 구조)
 * 
 * 핵심 로직:
 * - IMU 사용하지 않음 (순수 2D LiDAR 데이터)
 * - scan_accumulation_cloud 토픽에서 누적된 2D 스캔 사용
 * - 15개 방향으로 ray casting (P1과 동일하지만 입력 다름)
 */
class UnifiedScanAccumulatorPlanner : public BasePathPlanner
{
public:
    UnifiedScanAccumulatorPlanner() : BasePathPlanner("unified_scan_accumulator_planner", "/scan_accumulation_cloud")
    {
        // P2 고유 파라미터 (P1과 유사하지만 입력 토픽 다름)
        this->declare_parameter<int>("num_rays", 15);
        this->declare_parameter<double>("corridor_width", 0.15);  // 15cm corridor
        this->declare_parameter<double>("max_detection_range", 5.0);
        
        this->get_parameter("num_rays", num_rays_);
        this->get_parameter("corridor_width", corridor_width_);
        this->get_parameter("max_detection_range", max_detection_range_);
        
        RCLCPP_INFO(this->get_logger(), 
            "P2 플래너 초기화: %d rays, %.2fm corridor, %.1fm range (순수 2D)", 
            num_rays_, corridor_width_, max_detection_range_);
    }

protected:
    double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override
    {
        // 1. 입력 데이터는 이미 2D 스캔 누적 결과이므로 별도 전처리 불필요
        // (scan_accumulator_node에서 이미 2D 처리됨)
        
        // 2. Ray casting으로 각 방향별 거리 측정
        std::vector<double> depths(num_rays_, max_detection_range_);
        std::vector<int> evidences(num_rays_, 0);  // 각 ray별 포인트 수
        
        for (int i = 0; i < num_rays_; ++i) {
            // 각도 계산: [-π/2, π/2] → [-X 방향 기준으로 변환]
            double base_angle = (i * M_PI / (num_rays_ - 1)) - (M_PI / 2.0);
            double ray_angle = base_angle + M_PI;  // -X 전방 기준으로 회전
            
            Eigen::Vector2f ray_dir(cos(ray_angle), sin(ray_angle));
            double min_distance = max_detection_range_;
            int points_in_ray = 0;
            
            // 각 포인트가 현재 ray corridor 내에 있는지 확인
            for (const auto& point : *cloud) {
                Eigen::Vector2f point_vec(point.x, point.y);
                
                // Ray 방향으로의 투영 확인
                if (point_vec.dot(ray_dir) > 0) {
                    // Ray로부터의 수직 거리 계산
                    double perpendicular_dist = std::abs(
                        point.x * ray_dir.y() - point.y * ray_dir.x()
                    );
                    
                    // Corridor 내부에 있으면 거리 업데이트
                    if (perpendicular_dist < (corridor_width_ / 2.0)) {
                        points_in_ray++;
                        double point_distance = point_vec.norm();
                        min_distance = std::min(min_distance, point_distance);
                    }
                }
            }
            
            depths[i] = min_distance;
            evidences[i] = points_in_ray;
        }
        
        // 3. 원시 점수 계산 (거리 + 인접성)
        std::vector<double> raw_scores(num_rays_, 0.0);
        double w1 = 0.7, w2 = 0.3;  // 거리 vs 인접성 가중치
        
        for (int i = 0; i < num_rays_; ++i) {
            double prev_depth = (i > 0) ? depths[i-1] : depths[i];
            double next_depth = (i < num_rays_ - 1) ? depths[i+1] : depths[i];
            raw_scores[i] = w1 * depths[i] + w2 * (prev_depth + next_depth);
        }
        
        // 4. Evidence-based scoring 적용 (베이스 클래스에서)
        std::vector<double> final_scores;
        applyEvidenceScoring(raw_scores, evidences, final_scores);
        
        // 5. 최적 방향 선택
        int best_ray_idx = std::distance(final_scores.begin(), 
                                        std::max_element(final_scores.begin(), final_scores.end()));
        
        double ideal_angle = ((best_ray_idx * M_PI / (num_rays_ - 1)) - (M_PI / 2.0)) + M_PI;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "P2: ray %d/%d, 거리=%.2fm, 각도=%.1f° (순수2D)", 
            best_ray_idx, num_rays_, depths[best_ray_idx], ideal_angle * 180/M_PI);
        
        return ideal_angle;
    }
    
    void publishCustomVisualization(const std_msgs::msg::Header& header,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   double direction) override
    {
        // P2는 별도의 커스텀 시각화 없음 (공통 디버그 화살표만 사용)
        // scan_accumulation_cloud 자체가 시각화 역할
    }

private:
    // P2 고유 파라미터
    int num_rays_;
    double corridor_width_;
    double max_detection_range_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedScanAccumulatorPlanner>());
    rclcpp::shutdown();
    return 0;
}