#include "base_path_planner.hpp"

/**
 * @brief P3: 3D 반구형 Ray-Casting 경로 계획기 (통일된 구조)
 * 
 * 핵심 로직:
 * - 3D 포인트 클라우드를 그대로 사용 (진짜 3D!)
 * - 반구 형태로 3D ray들을 발사 (방위각 ±90°, 고도각 ±30°)
 * - 각 3D ray corridor 내 장애물 탐지
 * - 최적의 3D 방향을 2D 경로로 변환
 */
class UnifiedPathPlanner3DCorridor : public BasePathPlanner
{
public:
    UnifiedPathPlanner3DCorridor() : BasePathPlanner("unified_path_planner_3d_corridor", "/downsampled_cloud")
    {
        // P3 고유 파라미터
        this->declare_parameter<int>("num_rays", 30);  // 30개로 증가
        this->declare_parameter<double>("corridor_width", 0.30);  // 30cm corridor
        this->declare_parameter<double>("max_detection_range", 5.0);
        
        this->get_parameter("num_rays", num_rays_);
        this->get_parameter("corridor_width", corridor_width_);
        this->get_parameter("max_detection_range", max_detection_range_);
        
        // UP3 전용 3D ray-casting 시각화 publisher
        ray_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ray_casting", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "P3 플래너 초기화: %d rays, %.2fm corridor, %.1fm range", 
            num_rays_, corridor_width_, max_detection_range_);
    }

protected:
    double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override
    {
        // 1. 3D 반구형 ray-casting (진짜 3D!)
        std::vector<double> depths(num_rays_, max_detection_range_);
        std::vector<int> evidences(num_rays_, 0);  // 각 ray별 포인트 수
        
        // 3D ray 방향들 생성 (반구형)
        std::vector<Eigen::Vector3f> ray_directions;
        generateHemisphericalRays(ray_directions);
        
        for (int i = 0; i < num_rays_; ++i) {
            Eigen::Vector3f ray_dir = ray_directions[i];
            double min_distance = max_detection_range_;
            int points_in_3d_corridor = 0;
            
            // 각 3D 포인트가 현재 3D ray corridor 내에 있는지 확인
            for (const auto& point : *cloud) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                
                // Ray 방향으로의 투영 확인 (전방만)
                float projection = point_vec.dot(ray_dir);
                if (projection > 0) {
                    // 3D ray로부터의 수직 거리 계산
                    Eigen::Vector3f point_on_ray = ray_dir * projection;
                    Eigen::Vector3f perpendicular_vec = point_vec - point_on_ray;
                    double perpendicular_dist = perpendicular_vec.norm();
                    
                    // 3D corridor 내부에 있으면 거리 업데이트
                    if (perpendicular_dist < (corridor_width_ / 2.0)) {
                        points_in_3d_corridor++;
                        double point_distance = point_vec.norm();
                        min_distance = std::min(min_distance, point_distance);
                    }
                }
            }
            
            depths[i] = min_distance;
            evidences[i] = points_in_3d_corridor;
        }
        
        // 2. 원시 점수 계산 (거리 + 인접성)
        std::vector<double> raw_scores(num_rays_, 0.0);
        double w1 = 0.7, w2 = 0.3;  // 거리 vs 인접성 가중치
        
        for (int i = 0; i < num_rays_; ++i) {
            double prev_depth = (i > 0) ? depths[i-1] : depths[i];
            double next_depth = (i < num_rays_ - 1) ? depths[i+1] : depths[i];
            raw_scores[i] = w1 * depths[i] + w2 * (prev_depth + next_depth);
        }
        
        // 3. Evidence-based scoring 적용 (베이스 클래스에서)
        std::vector<double> final_scores;
        applyEvidenceScoring(raw_scores, evidences, final_scores);
        
        // 4. 최적 방향 선택
        int best_ray_idx = std::distance(final_scores.begin(), 
                                        std::max_element(final_scores.begin(), final_scores.end()));
        
        // 5. 3D ray 방향을 2D 각도로 변환 (XY 평면 투영)
        Eigen::Vector3f best_ray_3d = ray_directions[best_ray_idx];
        double ideal_angle = atan2(best_ray_3d.y(), best_ray_3d.x());
        
        // -X 전방 기준으로 이미 생성되었으므로 추가 변환 불필요
        
        RCLCPP_DEBUG(this->get_logger(), 
            "P3 (3D Ray-casting): ray %d/%d, 3D방향(%.2f,%.2f,%.2f), 거리=%.2fm, 각도=%.1f°", 
            best_ray_idx, num_rays_, best_ray_3d.x(), best_ray_3d.y(), best_ray_3d.z(),
            depths[best_ray_idx], ideal_angle * 180/M_PI);
        
        return ideal_angle;
    }
    
    void publishCustomVisualization(const std_msgs::msg::Header& header,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   double direction) override
    {
        // UP3 전용: 3D ray들을 실제 스캔 거리까지 시각화
        visualization_msgs::msg::MarkerArray ray_array;
        ray_array.markers.clear();
        
        // DELETE_ALL로 이전 마커 정리
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header = header;
        delete_marker.ns = "ray_casting";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        ray_array.markers.push_back(delete_marker);
        
        // 3D ray 방향들 생성
        std::vector<Eigen::Vector3f> ray_directions;
        generateHemisphericalRays(ray_directions);
        
        // 각 ray별로 실제 스캔 거리 계산
        std::vector<double> ray_distances(ray_directions.size(), max_detection_range_);
        
        for (size_t i = 0; i < ray_directions.size(); ++i) {
            const auto& ray_dir = ray_directions[i];
            double min_distance = max_detection_range_;
            
            // 실제 스캔된 포인트와의 충돌 검사
            for (const auto& point : *cloud) {
                Eigen::Vector3f point_vec(point.x, point.y, point.z);
                
                // Ray 방향으로의 투영 확인
                float projection = point_vec.dot(ray_dir);
                if (projection > 0) {
                    // 3D ray로부터의 수직 거리 계산
                    Eigen::Vector3f point_on_ray = ray_dir * projection;
                    Eigen::Vector3f perpendicular_vec = point_vec - point_on_ray;
                    double perpendicular_dist = perpendicular_vec.norm();
                    
                    // 3D corridor 내부에 있으면 거리 업데이트
                    if (perpendicular_dist < (corridor_width_ / 2.0)) {
                        double point_distance = point_vec.norm();
                        min_distance = std::min(min_distance, point_distance);
                    }
                }
            }
            ray_distances[i] = min_distance;
        }
        
        // 실제 거리까지 ray 화살표로 시각화 (듬성듬성)
        for (size_t i = 0; i < ray_directions.size(); i += 3) {  // 3개 중 1개만 (듬성듬성)
            const auto& ray_dir = ray_directions[i];
            double ray_length = ray_distances[i];
            
            visualization_msgs::msg::Marker ray_arrow;
            ray_arrow.header = header;
            ray_arrow.ns = "ray_casting";
            ray_arrow.id = static_cast<int>(i + 1);  // 1부터 시작 (0은 DELETE_ALL용)
            ray_arrow.type = visualization_msgs::msg::Marker::ARROW;
            ray_arrow.action = visualization_msgs::msg::Marker::ADD;
            ray_arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            // Ray 시작점과 실제 스캔 거리까지의 끝점
            ray_arrow.points.resize(2);
            ray_arrow.points[0].x = 0.0;
            ray_arrow.points[0].y = 0.0;
            ray_arrow.points[0].z = 0.0;
            
            ray_arrow.points[1].x = ray_length * ray_dir.x();
            ray_arrow.points[1].y = ray_length * ray_dir.y();
            ray_arrow.points[1].z = ray_length * ray_dir.z();
            
            ray_arrow.scale.x = 0.02;  // 선 두께
            ray_arrow.scale.y = 0.04; // 화살표 머리 폭
            ray_arrow.scale.z = 0.04; // 화살표 머리 높이
            
            // 색상: 초록색 (UP3 표시)
            ray_arrow.color.r = 0.0;
            ray_arrow.color.g = 0.8;
            ray_arrow.color.b = 0.2;
            ray_arrow.color.a = 0.6;
            
            ray_array.markers.push_back(ray_arrow);
        }
        
        // keepalive 마커 추가
        visualization_msgs::msg::Marker keepalive;
        keepalive.header = header;
        keepalive.ns = "ray_casting";
        keepalive.id = 999999;
        keepalive.type = visualization_msgs::msg::Marker::SPHERE;
        keepalive.action = visualization_msgs::msg::Marker::ADD;
        keepalive.scale.x = keepalive.scale.y = keepalive.scale.z = 0.001;
        keepalive.color.a = 0.0;
        keepalive.lifetime = rclcpp::Duration::from_seconds(0.3);
        ray_array.markers.push_back(keepalive);
        
        // Ray 시각화 발행 (UP3만의 고유한 3D 시각화)
        ray_publisher_->publish(ray_array);
        
        RCLCPP_DEBUG(this->get_logger(), "UP3: 3D ray 시각화 완료 (%zu개)", ray_directions.size());
    }

private:
    void generateHemisphericalRays(std::vector<Eigen::Vector3f>& ray_directions)
    {
        ray_directions.clear();
        ray_directions.reserve(num_rays_);
        
        // 반구형 ray 분포 생성
        // 방위각: 좌우 ±90도 (전방 180도)
        // 고도각: -X축에서 -Z축으로 60도 (수평에서 아래로)
        
        int azimuth_count = static_cast<int>(sqrt(num_rays_ * 1.5));  // 방위각 방향 개수
        int elevation_count = num_rays_ / azimuth_count;              // 고도각 방향 개수
        
        for (int i = 0; i < azimuth_count && ray_directions.size() < num_rays_; ++i) {
            for (int j = 0; j < elevation_count && ray_directions.size() < num_rays_; ++j) {
                // 방위각: -90° ~ +90° (전방 기준)
                double azimuth = (i * 180.0 / (azimuth_count - 1)) - 90.0;
                azimuth = azimuth * M_PI / 180.0;  // 라디안 변환
                
                // 고도각: 수평 기준에서 약간 아래로만 (0° ~ -15°)
                // j=0: 0도 (수평, -X 방향)
                // j=max: 15도 아래쪽 (약간만 아래로)
                double elevation_deg = -(j * 15.0 / (elevation_count - 1));
                double elevation = elevation_deg * M_PI / 180.0;  // 라디안 변환
                
                // 3D 단위벡터 생성 (구면 좌표 → 직교 좌표)
                // -X 전방 기준, 수평에서 약간 아래로만
                float x = -cos(elevation) * cos(azimuth);  // 전방(-X)
                float y = cos(elevation) * sin(azimuth);   // 좌우(Y)
                float z = sin(elevation);                  // 상하(Z), 약간 음수 = 약간 아래쪽
                
                ray_directions.emplace_back(x, y, z);
            }
        }
        
        // 정확히 num_rays_개가 되도록 조정
        ray_directions.resize(num_rays_);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "3D 반구형 ray 생성: %zu개 (방위각:%d × 고도각:%d)", 
            ray_directions.size(), azimuth_count, elevation_count);
    }

private:
    // P3 고유 파라미터
    int num_rays_;
    double corridor_width_;
    double max_detection_range_;
    
    // UP3 전용 ray-casting 시각화
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ray_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedPathPlanner3DCorridor>());
    rclcpp::shutdown();
    return 0;
}