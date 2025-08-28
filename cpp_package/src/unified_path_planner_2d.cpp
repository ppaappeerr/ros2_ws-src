#include "base_path_planner.hpp"

/**
 * @brief P1: 3D→2D 투영 방식 경로 계획기 (통일된 구조)
 * 
 * 핵심 로직:
 * - 3D 포인트를 Z=0으로 평면화
 * - 15개 방향으로 ray casting
 * - 15cm 폭 corridor 내 최단거리 탐지
 */
class UnifiedPathPlanner2D : public BasePathPlanner
{
public:
    UnifiedPathPlanner2D() : BasePathPlanner("unified_path_planner_2d", "/sweep_cloud_cpp")
    {
        // P1 고유 파라미터
        this->declare_parameter<int>("num_rays", 15);
        this->declare_parameter<double>("corridor_width", 0.15);  // 15cm corridor
        this->declare_parameter<double>("max_detection_range", 5.0);
        
        this->get_parameter("num_rays", num_rays_);
        this->get_parameter("corridor_width", corridor_width_);
        this->get_parameter("max_detection_range", max_detection_range_);
        
        RCLCPP_INFO(this->get_logger(), 
            "P1 플래너 초기화: %d rays, %.2fm corridor, %.1fm range", 
            num_rays_, corridor_width_, max_detection_range_);
    }

protected:
    double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override
    {
        // 1. 포인트 클라우드를 2D로 평면화 (Z=0)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_2d->reserve(cloud->size());
        
        for (const auto& point : *cloud) {
            pcl::PointXYZ flat_point;
            flat_point.x = point.x;
            flat_point.y = point.y;
            flat_point.z = 0.0;  // 강제로 2D화
            cloud_2d->points.push_back(flat_point);
        }
        cloud_2d->width = cloud_2d->points.size();
        cloud_2d->height = 1;
        cloud_2d->is_dense = true;
        
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
            for (const auto& point : *cloud_2d) {
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
        
        // 4. Evidence-based scoring 적용
        std::vector<double> final_scores;
        applyEvidenceScoring(raw_scores, evidences, final_scores);
        
        // 5. 최적 방향 선택
        int best_ray_idx = std::distance(final_scores.begin(), 
                                        std::max_element(final_scores.begin(), final_scores.end()));
        
        double ideal_angle = ((best_ray_idx * M_PI / (num_rays_ - 1)) - (M_PI / 2.0)) + M_PI;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "P1: ray %d/%d, 거리=%.2fm, 각도=%.1f°", 
            best_ray_idx, num_rays_, depths[best_ray_idx], ideal_angle * 180/M_PI);
        
        return ideal_angle;
    }
    
    void publishCustomVisualization(const std_msgs::msg::Header& header,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   double direction) override
    {
        // P1은 별도의 커스텀 시각화 없음 (공통 디버그 화살표만 사용)
        // 필요시 ray들을 시각화할 수 있지만 지금은 간소화
    }

private:
    // P1 고유 파라미터
    int num_rays_;
    double corridor_width_;
    double max_detection_range_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedPathPlanner2D>());
    rclcpp::shutdown();
    return 0;
}