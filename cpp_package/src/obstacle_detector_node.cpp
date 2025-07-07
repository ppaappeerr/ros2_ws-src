#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <deque>
#include <mutex>
#include <string> // <--- 이 헤더 파일이 누락되었습니다!

class ObstacleDetectorNode : public rclcpp::Node
{
public:
    ObstacleDetectorNode() : Node("obstacle_detector_node")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("input_cloud_topic", "/corrected_cloud");
        this->declare_parameter<std::string>("obstacle_cloud_topic", "/obstacles_cloud");
        this->declare_parameter<std::string>("ground_cloud_topic", "/ground_cloud");
        this->declare_parameter<double>("accumulation_duration_sec", 0.5);
        this->declare_parameter<double>("processing_interval_sec", 0.2);
        this->declare_parameter<double>("ground_distance_threshold", 0.05);

        accumulation_duration_ = this->get_parameter("accumulation_duration_sec").as_double();
        processing_interval_ = this->get_parameter("processing_interval_sec").as_double();
        ground_distance_threshold_ = this->get_parameter("ground_distance_threshold").as_double();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(20));

        // Deskew된 포인트 클라우드 구독
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("input_cloud_topic").as_string(), qos,
            std::bind(&ObstacleDetectorNode::cloudCallback, this, std::placeholders::_1));

        // 장애물 포인트 클라우드 발행
        obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("obstacle_cloud_topic").as_string(), 10);
            
        // (디버깅용) 바닥 포인트 클라우드 발행
        ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            this->get_parameter("ground_cloud_topic").as_string(), 10);
            
        // 주기적으로 누적/처리 함수를 호출할 타이머
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(processing_interval_ * 1000)),
            std::bind(&ObstacleDetectorNode::processAccumulatedCloud, this));

        RCLCPP_INFO(this->get_logger(), "Obstacle Detector Node has started.");
    }

private:
    using PointType = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointType>;

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(cloud_buffer_mutex_);
        cloud_buffer_.push_back(*msg);
    }

    void processAccumulatedCloud()
    {
        std::deque<sensor_msgs::msg::PointCloud2> current_clouds;
        {
            std::lock_guard<std::mutex> lock(cloud_buffer_mutex_);
            if (cloud_buffer_.empty()) {
                return;
            }
            
            // accumulation_duration 이전 데이터는 삭제
            rclcpp::Time cutoff_time = this->now() - rclcpp::Duration::from_seconds(accumulation_duration_);
            while (!cloud_buffer_.empty() && rclcpp::Time(cloud_buffer_.front().header.stamp) < cutoff_time) {
                cloud_buffer_.pop_front();
            }
            
            current_clouds = cloud_buffer_; // 현재 버퍼의 모든 클라우드 복사
        }

        if (current_clouds.empty()) return;

        // 1. 모든 클라우드를 하나로 합치기
        PointCloud::Ptr accumulated_cloud(new PointCloud());
        for (const auto& cloud_msg : current_clouds) {
            PointCloud temp_cloud;
            pcl::fromROSMsg(cloud_msg, temp_cloud);
            *accumulated_cloud += temp_cloud;
        }

        if (accumulated_cloud->empty()) return;

        // 2. Downsampling (선택 사항이지만 성능에 도움)
        PointCloud::Ptr downsampled_cloud(new PointCloud());
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(accumulated_cloud);
        sor.setLeafSize(0.02f, 0.02f, 0.02f);
        sor.filter(*downsampled_cloud);

        // 3. 바닥 평면 제거 (RANSAC)
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(ground_distance_threshold_);

        seg.setInputCloud(downsampled_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            // 바닥면을 못 찾았을 경우, 전체를 장애물로 간주하고 발행
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Could not estimate a planar model. Publishing all points as obstacles.");
            sensor_msgs::msg::PointCloud2 obstacle_msg;
            pcl::toROSMsg(*downsampled_cloud, obstacle_msg);
            obstacle_msg.header = current_clouds.back().header;
            obstacle_pub_->publish(obstacle_msg);
            return;
        }

        // 4. 바닥(inliers)과 장애물(outliers) 분리
        PointCloud::Ptr ground_cloud(new PointCloud);
        PointCloud::Ptr obstacle_cloud(new PointCloud);
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(downsampled_cloud);
        extract.setIndices(inliers);
        
        // 바닥 추출
        extract.setNegative(false);
        extract.filter(*ground_cloud);
        
        // 장애물 추출
        extract.setNegative(true);
        extract.filter(*obstacle_cloud);

        // 5. 결과 발행
        sensor_msgs::msg::PointCloud2 obstacle_msg, ground_msg;
        
        pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
        obstacle_msg.header = current_clouds.back().header;
        obstacle_pub_->publish(obstacle_msg);

        pcl::toROSMsg(*ground_cloud, ground_msg);
        ground_msg.header = current_clouds.back().header;
        ground_pub_->publish(ground_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::deque<sensor_msgs::msg::PointCloud2> cloud_buffer_;
    std::mutex cloud_buffer_mutex_;

    double accumulation_duration_;
    double processing_interval_;
    double ground_distance_threshold_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}