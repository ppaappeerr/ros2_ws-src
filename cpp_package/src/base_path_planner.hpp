#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "angles/angles.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <string>

/**
 * @brief 모든 경로 계획 알고리즘의 베이스 클래스
 * 
 * 공통 기능:
 * - Dead-zone 필터링 (어깨 장착용)
 * - ROI 반경 필터링
 * - Angular velocity 제한
 * - Evidence-based scoring
 * - 통일된 스무딩 로직
 * - 동일한 파라미터 구조
 */
class BasePathPlanner : public rclcpp::Node
{
public:
    BasePathPlanner(const std::string& node_name, const std::string& input_topic);
    
protected:
    /**
     * @brief 공통 파라미터 선언 및 초기화
     */
    void declareCommonParameters();
    
    /**
     * @brief 포인트 클라우드 전처리 (Dead-zone + ROI 필터링)
     * @param input 입력 포인트 클라우드
     * @param output 필터링된 포인트 클라우드
     */
    void preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& output);
    
    /**
     * @brief Evidence-based scoring 적용
     * @param raw_scores 원본 점수들
     * @param evidences 각 방향의 증거 수 (포인트 개수)
     * @param final_scores 최종 점수들 (출력)
     */
    void applyEvidenceScoring(const std::vector<double>& raw_scores,
                             const std::vector<int>& evidences,
                             std::vector<double>& final_scores);
    
    /**
     * @brief 각속도 제한 적용한 스무딩
     * @param ideal_angle 목표 각도
     * @param dt 시간 간격
     * @return 스무딩된 각도
     */
    double applySmoothingWithAngularLimit(double ideal_angle, double dt);
    
    /**
     * @brief 안전 벡터 발행
     * @param header 메시지 헤더
     * @param angle 방향 각도
     */
    void publishSafeVector(const std_msgs::msg::Header& header, double angle);
    
    /**
     * @brief 공통 디버그 마커 발행 (최종 방향 화살표)
     * @param header 메시지 헤더  
     * @param angle 방향 각도
     */
    void publishDebugVector(const std_msgs::msg::Header& header, double angle);
    
    /**
     * @brief 각 플래너가 구현해야 하는 핵심 경로 계획 로직
     * @param cloud 전처리된 포인트 클라우드
     * @return 목표 방향 각도 (라디안, -X 방향 기준)
     */
    virtual double computePathDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) = 0;
    
    /**
     * @brief 각 플래너별 고유 시각화 (선택사항)
     * @param header 메시지 헤더
     * @param cloud 포인트 클라우드
     * @param direction 계산된 방향
     */
    virtual void publishCustomVisualization(const std_msgs::msg::Header& /* header */,
                                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& /* cloud */,
                                           double /* direction */) {}

private:
    /**
     * @brief 포인트 클라우드 콜백 함수
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

protected:
    // === 공통 파라미터 ===
    // Dead-zone 및 ROI
    bool dead_zone_enabled_;
    std::string mount_side_;
    double dead_zone_forward_x_;
    double dead_zone_lateral_y_;
    double roi_max_radius_;
    
    // Evidence-based scoring
    double unknown_evidence_k_;
    
    // Angular smoothing
    double max_angular_velocity_;
    bool use_adaptive_smoothing_;
    double urgency_threshold_;
    double urgency_multiplier_;
    
    // 상태 변수
    double smoothed_angle_;
    rclcpp::Time last_time_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_publisher_;
    
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};