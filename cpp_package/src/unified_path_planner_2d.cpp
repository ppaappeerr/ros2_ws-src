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
    UnifiedPathPlanner2D() : BasePathPlanner("unified_path_planner_2d", "/downsampled_cloud")
    {
        // P1 고유 파라미터
        this->declare_parameter<int>("num_rays", 15);
        this->declare_parameter<double>("corridor_width", 0.15);  // 15cm corridor
        this->declare_parameter<double>("max_detection_range", 5.0);
        
        this->get_parameter("num_rays", num_rays_);
        this->get_parameter("corridor_width", corridor_width_);
        this->get_parameter("max_detection_range", max_detection_range_);
        
        // 2D 사영된 포인트클라우드 발행용 publisher 추가 (UP2와 동일한 토픽명)
        projected_2d_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_accumulation_cloud", 10);
        
        // UP1 전용 ray-casting 시각화 publisher
        ray_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ray_casting", 10);
        
        RCLCPP_INFO(this->get_logger(), 
            "UP1 플래너 초기화: 3D→2D 사영 + 2D ray-casting, %d rays, %.2fm corridor, %.1fm range", 
            num_rays_, corridor_width_, max_detection_range_);
    }

protected:
    double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) override
    {
        // 1. 3D 포인트 클라우드를 2D로 사영 (Z=0으로 평면화)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_2d->reserve(cloud->size());
        
        for (const auto& point : *cloud) {
            pcl::PointXYZ flat_point;
            flat_point.x = point.x;
            flat_point.y = point.y;
            flat_point.z = 0.0;  // 강제로 2D 평면화
            cloud_2d->points.push_back(flat_point);
        }
        cloud_2d->width = cloud_2d->points.size();
        cloud_2d->height = 1;
        cloud_2d->is_dense = true;
        
        // 2D 사영된 포인트클라우드 발행 (시각화용)
        if (cloud_2d->points.size() > 0) {
            sensor_msgs::msg::PointCloud2 output_msg;
            pcl::toROSMsg(*cloud_2d, output_msg);
            output_msg.header.stamp = this->get_clock()->now();
            output_msg.header.frame_id = "base_link";
            projected_2d_publisher_->publish(output_msg);
        }
        
        // 2. 2D Ray casting으로 각 방향별 거리 측정
        std::vector<double> depths(num_rays_, max_detection_range_);
        std::vector<int> evidences(num_rays_, 0);  // 각 ray별 포인트 수
        
        for (int i = 0; i < num_rays_; ++i) {
            // 각도 계산: [-π/2, π/2] → [-X 방향 기준으로 변환]
            double base_angle = (i * M_PI / (num_rays_ - 1)) - (M_PI / 2.0);
            double ray_angle = base_angle + M_PI;  // -X 전방 기준으로 회전
            
            Eigen::Vector2f ray_dir(cos(ray_angle), sin(ray_angle));
            double min_distance = max_detection_range_;
            int points_in_ray = 0;
            
            // 각 2D 포인트가 현재 ray corridor 내에 있는지 확인
            for (const auto& point : *cloud_2d) {
                Eigen::Vector2f point_vec(point.x, point.y);
                
                // Ray 방향으로의 투영 확인 (전방만)
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
            "UP1 (2D Ray-casting): ray %d/%d, 거리=%.2fm, 각도=%.1f°, 2D포인트=%zu개", 
            best_ray_idx, num_rays_, depths[best_ray_idx], ideal_angle * 180/M_PI, cloud_2d->points.size());
        
        return ideal_angle;
    }
    
    void publishCustomVisualization(const std_msgs::msg::Header& header,
                                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   double direction) override
    {
        // UP1 전용: 2D ray들을 실제 스캔 거리까지 시각화
        visualization_msgs::msg::MarkerArray ray_array;
        ray_array.markers.clear();
        
        // DELETE_ALL로 이전 마커 정리
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header = header;
        delete_marker.ns = "ray_casting";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        ray_array.markers.push_back(delete_marker);
        
        // 각 ray별로 실제 스캔 거리 계산
        std::vector<double> ray_distances(num_rays_, max_detection_range_);
        
        for (int i = 0; i < num_rays_; ++i) {
            double base_angle = (i * M_PI / (num_rays_ - 1)) - (M_PI / 2.0);
            double ray_angle = base_angle + M_PI;  // -X 전방 기준
            
            Eigen::Vector2f ray_dir(cos(ray_angle), sin(ray_angle));
            double min_distance = max_detection_range_;
            
            // 실제 스캔된 포인트와의 충돌 검사 (2D 사영된 포인트 사용)
            for (const auto& point : *cloud) {
                Eigen::Vector2f point_vec(point.x, point.y);  // Z 무시
                
                // Ray 방향으로의 투영 확인 (전방만)
                if (point_vec.dot(ray_dir) > 0) {
                    // Ray로부터의 수직 거리 계산
                    double perpendicular_dist = std::abs(
                        point.x * ray_dir.y() - point.y * ray_dir.x()
                    );
                    
                    // Corridor 내부에 있으면 거리 업데이트
                    if (perpendicular_dist < (corridor_width_ / 2.0)) {
                        double point_distance = point_vec.norm();
                        min_distance = std::min(min_distance, point_distance);
                    }
                }
            }
            ray_distances[i] = min_distance;
        }
        
        // 실제 거리까지 ray 시각화
        for (int i = 0; i < num_rays_; ++i) {
            double base_angle = (i * M_PI / (num_rays_ - 1)) - (M_PI / 2.0);
            double ray_angle = base_angle + M_PI;  // -X 전방 기준
            double ray_length = ray_distances[i];
            
            visualization_msgs::msg::Marker ray_line;
            ray_line.header = header;
            ray_line.ns = "ray_casting";
            ray_line.id = i + 1;  // 1부터 시작 (0은 DELETE_ALL용)
            ray_line.type = visualization_msgs::msg::Marker::ARROW;
            ray_line.action = visualization_msgs::msg::Marker::ADD;
            ray_line.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            // Ray 시작점과 실제 스캔 거리까지의 끝점
            ray_line.points.resize(2);
            ray_line.points[0].x = 0.0;
            ray_line.points[0].y = 0.0;
            ray_line.points[0].z = 0.05; // 바닥 위 5cm
            
            ray_line.points[1].x = ray_length * cos(ray_angle);
            ray_line.points[1].y = ray_length * sin(ray_angle);
            ray_line.points[1].z = 0.05;
            
            ray_line.scale.x = 0.02;  // 선 두께
            ray_line.scale.y = 0.03; // 화살표 머리 폭
            ray_line.scale.z = 0.03; // 화살표 머리 높이
            
            // 색상: 매우 연한 파란색 (2D ray를 나타냄, 배경용)
            ray_line.color.r = 0.4;
            ray_line.color.g = 0.6;
            ray_line.color.b = 0.8;
            ray_line.color.a = 0.15;  // 더 연하게
            
            ray_array.markers.push_back(ray_line);
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
        
        // Ray 시각화 발행 (UP1만의 고유한 시각화)
        ray_publisher_->publish(ray_array);
        
        RCLCPP_DEBUG(this->get_logger(), "UP1: 2D ray 시각화 완료 (%d개)", num_rays_);
    }

private:
    // UP1 고유 파라미터
    int num_rays_;
    double corridor_width_;
    double max_detection_range_;
    
    // 2D 사영된 포인트클라우드 발행용
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr projected_2d_publisher_;
    
    // UP1 전용 ray-casting 시각화
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ray_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnifiedPathPlanner2D>());
    rclcpp::shutdown();
    return 0;
}