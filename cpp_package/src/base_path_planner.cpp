#include "base_path_planner.hpp"

BasePathPlanner::BasePathPlanner(const std::string& node_name, const std::string& input_topic)
    : Node(node_name), smoothed_angle_(M_PI) // 초기 방향: -X (전방)
{
    // 공통 파라미터 선언
    declareCommonParameters();
    
    // 구독자 설정
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, 
        std::bind(&BasePathPlanner::pointCloudCallback, this, std::placeholders::_1));
    
    // 발행자 설정
    vector_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/safe_path_vector", 10);
    debug_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/path_planner_debug", 10);
    
    last_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "%s 시작됨 (전방 = -X)", node_name.c_str());
}

void BasePathPlanner::declareCommonParameters()
{
    // === Dead-zone 및 ROI 파라미터 ===
    this->declare_parameter<bool>("dead_zone_enabled", true);
    this->declare_parameter<std::string>("mount_side", "right");
    this->declare_parameter<double>("dead_zone_forward_x", 0.25);  // 전방 25cm
    this->declare_parameter<double>("dead_zone_lateral_y", 0.50);  // 측면 50cm
    this->declare_parameter<double>("roi_max_radius", 5.0);        // 최대 5m (수정됨)

    this->get_parameter("dead_zone_enabled", dead_zone_enabled_);
    this->get_parameter("mount_side", mount_side_);
    this->get_parameter("dead_zone_forward_x", dead_zone_forward_x_);
    this->get_parameter("dead_zone_lateral_y", dead_zone_lateral_y_);
    this->get_parameter("roi_max_radius", roi_max_radius_);

    // === Evidence-based scoring ===
    this->declare_parameter<double>("unknown_evidence_k", 5.0);    // 편향 보정 상수
    this->get_parameter("unknown_evidence_k", unknown_evidence_k_);

    // === Angular smoothing ===
    this->declare_parameter<double>("max_angular_velocity", 1.57);  // 90 deg/s
    this->declare_parameter<bool>("use_adaptive_smoothing", true);
    this->declare_parameter<double>("urgency_threshold", 0.524);    // 30도
    this->declare_parameter<double>("urgency_multiplier", 1.5);     // 긴급시 1.5배 빨라짐

    this->get_parameter("max_angular_velocity", max_angular_velocity_);
    this->get_parameter("use_adaptive_smoothing", use_adaptive_smoothing_);
    this->get_parameter("urgency_threshold", urgency_threshold_);
    this->get_parameter("urgency_multiplier", urgency_multiplier_);

    RCLCPP_INFO(this->get_logger(), 
        "공통 파라미터 로드됨: dead_zone=%s, mount_side=%s, roi_radius=%.2fm",
        dead_zone_enabled_ ? "활성화" : "비활성화",
        mount_side_.c_str(), roi_max_radius_);
}

void BasePathPlanner::preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
    output->clear();
    output->reserve(input->size());

    for (const auto& point : *input) {
        // 1. 전방 뷰 필터링 (x > 0은 후방이므로 제외)
        if (point.x > 0) continue;
        
        // 2. ROI 반경 필터링
        double distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance > roi_max_radius_) continue;
        
        // 3. Dead-zone 필터링 (어깨 장착용)
        if (dead_zone_enabled_) {
            bool in_dead_zone = false;
            
            // 전방 dead zone 체크 (x < 0 && x > -dead_zone_forward_x_)
            if (point.x < 0 && point.x > -dead_zone_forward_x_) {
                if (mount_side_ == "right" && point.y > 0 && point.y < dead_zone_lateral_y_) {
                    // 오른쪽 어깨 장착, 왼쪽(+y) dead zone
                    in_dead_zone = true;
                } else if (mount_side_ == "left" && point.y < 0 && point.y > -dead_zone_lateral_y_) {
                    // 왼쪽 어깨 장착, 오른쪽(-y) dead zone  
                    in_dead_zone = true;
                }
            }
            
            if (in_dead_zone) continue;
        }
        
        output->points.push_back(point);
    }
    
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
    
    RCLCPP_DEBUG(this->get_logger(), 
        "전처리 완료: %zu → %zu points", input->size(), output->size());
}

void BasePathPlanner::applyEvidenceScoring(const std::vector<double>& raw_scores,
                                          const std::vector<int>& evidences,
                                          std::vector<double>& final_scores)
{
    final_scores.resize(raw_scores.size());
    
    for (size_t i = 0; i < raw_scores.size(); ++i) {
        // Evidence factor: 실제 포인트가 많을수록 신뢰도 증가
        double evidence_factor = static_cast<double>(evidences[i]) / 
                                (evidences[i] + unknown_evidence_k_);
        
        final_scores[i] = raw_scores[i] * evidence_factor;
        
        // Edge penalty: 극단 방향(±90도) 회피
        size_t num_directions = raw_scores.size();
        if (i == 0 || i == num_directions - 1) {
            final_scores[i] *= 0.5;  // 강한 페널티
        } else if (i == 1 || i == num_directions - 2) {
            final_scores[i] *= 0.8;  // 중간 페널티
        }
        
        // Central bias: 중앙 선호 (FOV 가장자리 회피)
        double center_idx = (num_directions - 1) / 2.0;
        double norm_pos = std::abs(static_cast<double>(i) - center_idx) / center_idx;
        final_scores[i] *= (1.0 - 0.15 * norm_pos * norm_pos);
    }
}

double BasePathPlanner::applySmoothingWithAngularLimit(double ideal_angle, double dt)
{
    double angle_diff = angles::shortest_angular_distance(smoothed_angle_, ideal_angle);
    
    double effective_max_angular_velocity = max_angular_velocity_;
    
    // 적응형 스무딩: 큰 방향 변화 시 더 빠르게 반응
    if (use_adaptive_smoothing_ && std::abs(angle_diff) > urgency_threshold_) {
        effective_max_angular_velocity *= urgency_multiplier_;
        RCLCPP_DEBUG(this->get_logger(), 
            "긴급 상황 감지: 각속도 %.1f → %.1f deg/s", 
            max_angular_velocity_ * 180/M_PI, 
            effective_max_angular_velocity * 180/M_PI);
    }
    
    double max_angle_change = effective_max_angular_velocity * dt;
    double angle_change = std::clamp(angle_diff, -max_angle_change, max_angle_change);
    
    smoothed_angle_ = angles::normalize_angle(smoothed_angle_ + angle_change);
    
    return smoothed_angle_;
}

void BasePathPlanner::publishSafeVector(const std_msgs::msg::Header& header, double angle)
{
    geometry_msgs::msg::Vector3Stamped safe_vector_msg;
    safe_vector_msg.header = header;
    safe_vector_msg.vector.x = cos(angle);
    safe_vector_msg.vector.y = sin(angle);
    safe_vector_msg.vector.z = 0;
    vector_publisher_->publish(safe_vector_msg);
}

void BasePathPlanner::publishDebugVector(const std_msgs::msg::Header& header, double angle)
{
    visualization_msgs::msg::MarkerArray marker_array;
    
    // 최종 방향 화살표
    visualization_msgs::msg::Marker arrow;
    arrow.header = header;
    arrow.ns = "safe_direction";
    arrow.id = 0;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.lifetime = rclcpp::Duration::from_seconds(0.2);
    
    arrow.points.resize(2);
    arrow.points[0].x = 0;
    arrow.points[0].y = 0;
    arrow.points[0].z = 0.1;
    arrow.points[1].x = 2.0 * cos(angle);
    arrow.points[1].y = 2.0 * sin(angle);
    arrow.points[1].z = 0.1;
    
    arrow.scale.x = 0.05;  // 화살표 두께
    arrow.scale.y = 0.1;   // 화살표 폭
    arrow.scale.z = 0.15;  // 화살표 높이
    
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.color.a = 0.9;
    
    marker_array.markers.push_back(arrow);
    debug_publisher_->publish(marker_array);
}

void BasePathPlanner::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    
    // PCL 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    
    // 빈 클라우드 체크
    if (cloud->empty()) {
        RCLCPP_DEBUG(this->get_logger(), "빈 포인트 클라우드 수신됨");
        return;
    }
    
    // 공통 전처리
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    preprocessPointCloud(cloud, filtered_cloud);
    
    // 전처리 후에도 빈 클라우드면 전방 직진
    if (filtered_cloud->empty()) {
        RCLCPP_DEBUG(this->get_logger(), "전처리 후 빈 클라우드 - 전방 직진");
        double final_angle = applySmoothingWithAngularLimit(M_PI, dt);
        publishSafeVector(msg->header, final_angle);
        publishDebugVector(msg->header, final_angle);
        return;
    }
    
    // 각 플래너별 핵심 로직 실행
    double ideal_angle = computePathDirection(filtered_cloud);
    
    // 공통 스무딩 적용
    double final_angle = applySmoothingWithAngularLimit(ideal_angle, dt);
    
    // 결과 발행
    publishSafeVector(msg->header, final_angle);
    publishDebugVector(msg->header, final_angle);
    
    // 플래너별 커스텀 시각화
    publishCustomVisualization(msg->header, filtered_cloud, final_angle);
}