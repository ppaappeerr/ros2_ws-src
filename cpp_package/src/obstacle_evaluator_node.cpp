#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>
#include "cpp_package/msg/obstacle_feedback.hpp"

class ObstacleEvaluator : public rclcpp::Node {
public:
  ObstacleEvaluator() : Node("obstacle_evaluator")
  {
    this->declare_parameter("center_width_deg", 30.0);  // ±15°
    this->declare_parameter("danger_dist", 1.2);        // 1.2 m
    this->declare_parameter("warn_dist", 2.0);           // 2.0 m
    
    this->get_parameter("center_width_deg", cwidth_);
    this->get_parameter("danger_dist", danger_);
    this->get_parameter("warn_dist", warn_);

    double halfF = (cwidth_ * M_PI / 180.0) / 2.0;
    tan_center_ = std::tan(halfF);

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/front_cloud", rclcpp::SensorDataQoS(),
      std::bind(&ObstacleEvaluator::cbCloud, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<cpp_package::msg::ObstacleFeedback>(
      "/obstacle_feedback", 10);
      
    RCLCPP_INFO(this->get_logger(), "Obstacle Evaluator started");
    RCLCPP_INFO(this->get_logger(), "Center width: %.1f deg, Danger: %.1f m, Warn: %.1f m", 
                cwidth_, danger_, warn_);
  }

private:
  void cbCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pc);
    
    if (pc->points.empty()) {
      return;
    }
    
    float min_l = 9e9, min_c = 9e9, min_r = 9e9;

    for (const auto &p: pc->points) {
      float dist = std::hypot(p.x, p.y);
      if (p.y > 0) min_l = std::min(min_l, dist);       // 왼쪽(+y)
      if (p.y < 0) min_r = std::min(min_r, dist);       // 오른쪽(-y)
      if (std::fabs(p.y) < tan_center_ * p.x)
        min_c = std::min(min_c, dist);                  // 정면
    }

    auto feedback = cpp_package::msg::ObstacleFeedback();
    feedback.header.stamp = msg->header.stamp;
    feedback.header.frame_id = msg->header.frame_id;
    feedback.min_left     = std::isfinite(min_l) ? min_l : 99.0f;
    feedback.min_right    = std::isfinite(min_r) ? min_r : 99.0f;
    feedback.min_center   = std::isfinite(min_c) ? min_c : 99.0f;

    // 간단 위험도→레벨 (0-2)
    auto lv = [&](float d) -> uint8_t {
      return (d < danger_) ? 2 : (d < warn_) ? 1 : 0;
    };
    feedback.level_left   = lv(feedback.min_left);
    feedback.level_center = lv(feedback.min_center);
    feedback.level_right  = lv(feedback.min_right);

    pub_->publish(feedback);
    
    RCLCPP_DEBUG(this->get_logger(), 
                "L:%.1f(%d) C:%.1f(%d) R:%.1f(%d)",
                feedback.min_left, feedback.level_left,
                feedback.min_center, feedback.level_center,
                feedback.min_right, feedback.level_right);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<cpp_package::msg::ObstacleFeedback>::SharedPtr pub_;
  double cwidth_, danger_, warn_, tan_center_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleEvaluator>());
  rclcpp::shutdown();
  return 0;
}
