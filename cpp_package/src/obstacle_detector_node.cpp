#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <memory>

// PCL 포인트 클라우드 타입 정의
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

class ObstacleDetectorNode : public rclcpp::Node {
public:
    ObstacleDetectorNode() : Node("obstacle_detector_node") {
        // RViz나 launch 파일에서 쉽게 값을 바꿀 수 있도록 파라미터 선언
        this->declare_parameter<double>("ground_z_min", -0.5); // 지면 후보 필터링 최소 높이 (센서 기준)
        this->declare_parameter<double>("ground_z_max", -0.3); // 지면 후보 필터링 최대 높이 (센서 기준)
        this->declare_parameter<double>("plane_distance_threshold", 0.05); // RANSAC 평면 모델 허용 오차 (m)
        this->declare_parameter<int>("min_points_for_plane", 50); // 평면 추정을 위한 최소 포인트 수

        // /points_3d 토픽을 구독하고, 메시지가 수신되면 cloud_callback 함수를 실행
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_3d", 10, std::bind(&ObstacleDetectorNode::cloud_callback, this, std::placeholders::_1));

        // 결과를 발행할 퍼블리셔 생성
        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 10);
        obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacles_cloud", 10);

        RCLCPP_INFO(this->get_logger(), "Obstacle Detector Node has been started.");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. ROS 메시지를 PCL 포인트 클라우드로 변환
        PointCloud::Ptr cloud(new PointCloud);
        pcl::fromROSMsg(*msg, *cloud);

        // 포인트 클라우드가 비어있으면 아무 작업도 하지 않음
        if (cloud->points.empty()) {
            return;
        }

        // 2. PassThrough 필터로 관심 영역(ROI) 설정 (지면 후보군 추출)
        PointCloud::Ptr cloud_filtered(new PointCloud);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z"); // Z축 기준으로 필터링
        pass.setFilterLimits(this->get_parameter("ground_z_min").as_double(), this->get_parameter("ground_z_max").as_double());
        pass.filter(*cloud_filtered);
        
        // 지면 후보 점들이 너무 적으면 평면 추정이 무의미하므로, 원본을 그대로 장애물로 판단하고 종료
        if (cloud_filtered->points.size() < this->get_parameter("min_points_for_plane").as_int()) {
             RCLCPP_WARN(this->get_logger(), "Not enough points in the filtered cloud to detect a ground plane.");
             obstacle_pub_->publish(*msg);
             return;
        }

        // 3. RANSAC으로 평면 모델 추정
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);      // 최적화 활성화
        seg.setModelType(pcl::SACMODEL_PLANE);  // 평면 모델 사용
        seg.setMethodType(pcl::SAC_RANSAC);     // RANSAC 알고리즘 사용
        seg.setDistanceThreshold(this->get_parameter("plane_distance_threshold").as_double());
        seg.setInputCloud(cloud_filtered);      // 필터링된 지면 후보군을 입력으로 사용
        seg.segment(*inliers, *coefficients);

        // 평면을 찾지 못했다면(inlier가 없다면), 원본을 그대로 장애물로 판단하고 종료
        if (inliers->indices.empty()) {
            RCLCPP_WARN(this->get_logger(), "Could not estimate a planar model for the given cloud.");
            obstacle_pub_->publish(*msg);
            return;
        }

        // 4. 원본 클라우드에서 지면(inliers)과 장애물(outliers) 분리
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);       // **중요: 분리는 원본 클라우드에서 수행**
        extract.setIndices(inliers);        // RANSAC으로 찾은 지면 점들의 인덱스 설정

        // 지면 포인트 클라우드 추출
        PointCloud::Ptr ground_cloud(new PointCloud);
        extract.setNegative(false); // inlier(지면) 추출
        extract.filter(*ground_cloud);

        // 장애물 포인트 클라우드 추출
        PointCloud::Ptr obstacle_cloud(new PointCloud);
        extract.setNegative(true);  // outlier(장애물) 추출
        extract.filter(*obstacle_cloud);
        
        // 5. 결과를 다시 ROS 메시지로 변환하여 발행
        sensor_msgs::msg::PointCloud2 ground_msg, obstacle_msg;
        pcl::toROSMsg(*ground_cloud, ground_msg);
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        ground_msg.header = msg->header;
        obstacle_msg.header = msg->header;

        ground_pub_->publish(ground_msg);
        obstacle_pub_->publish(obstacle_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}