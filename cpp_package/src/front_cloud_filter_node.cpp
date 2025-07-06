#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <cmath>  // 🔥 M_PI 정의를 위해 추가

class FrontCloudFilter : public rclcpp::Node {
public:
  FrontCloudFilter() : Node("front_cloud_filter") {
    // 파라미터 선언 (기본값 설정)
    double fov_deg = this->declare_parameter<double>("field_of_view_deg", 120.0);
    double max_dist = this->declare_parameter<double>("max_distance", 3.0);
    std::string input_topic = this->declare_parameter<std::string>("input_topic", "/sweep_cloud");
    std::string output_topic = this->declare_parameter<std::string>("output_topic", "/front_cloud");
    double voxel_size = this->declare_parameter<double>("voxel_leaf_size", 0.05);

    // 퍼블리셔 설정
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    // 구독자 설정 (SensorDataQoS 사용하여 신뢰도 향상)
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&FrontCloudFilter::pointCloudCallback, this, std::placeholders::_1)
    );

    // 각도/거리 한계 사전 계산
    half_fov_rad_ = (fov_deg * M_PI / 180.0) / 2.0;        // 절반 각도 (라디안)
    tan_half_fov_ = std::tan(half_fov_rad_);               // tan(half_fov)
    max_dist_sq_ = max_dist * max_dist;                    // 최대 거리^2 (비교용)
    voxel_leaf_size_ = static_cast<float>(voxel_size);     // 🔥 float로 캐스팅

    RCLCPP_INFO(this->get_logger(), "Front Cloud Filter Node started");
    RCLCPP_INFO(this->get_logger(), "FOV: %.1f deg, Max distance: %.1f m", fov_deg, max_dist);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 1. sensor_msgs::PointCloud2 → PCL PointCloud 변환
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
      return;
    }

    // 2. 전방 시야 및 거리 필터링
    pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    front_cloud->points.reserve(cloud->points.size());
    
    for (const auto &pt : cloud->points) {
      float x = pt.x;
      float y = pt.y;
      float z = pt.z;
      
      // 전방(±half_fov) & 최대거리 조건 체크
      if (x > 0.0f && 
          (y * y <= (tan_half_fov_ * x) * (tan_half_fov_ * x)) && 
          ((x * x + y * y + z * z) <= max_dist_sq_)) {
        front_cloud->points.push_back(pt);
      }
    }
    
    front_cloud->width = front_cloud->points.size();
    front_cloud->height = 1;
    front_cloud->is_dense = true;

    // 3. VoxelGrid 다운샘플링
    if (front_cloud->points.size() > 1) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_down(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(front_cloud);
      voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel.filter(*filtered_down);
      front_cloud.swap(filtered_down);
    }

    // 4. PCL → sensor_msgs::PointCloud2 변환 후 퍼블리시
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*front_cloud, output_msg);
    output_msg.header = msg->header;  // 입력 헤더(프레임ID, timestamp) 유지
    pub_->publish(output_msg);

    // 🔥 디버그 정보
    RCLCPP_DEBUG(this->get_logger(), 
                "Filtered: %zu -> %zu points", 
                cloud->points.size(), front_cloud->points.size());
  }

  // 멤버 변수
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  double half_fov_rad_;
  double tan_half_fov_;
  double max_dist_sq_;
  float voxel_leaf_size_;
};

// 노드 실행을 위한 main 함수
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
