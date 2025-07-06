#include <rclcpp/rclcpp.hpp>
#include "cpp_package/msg/obstacle_feedback.hpp"
#include "cpp_package/msg/nav_hint.hpp"

class NavHint : public rclcpp::Node {
public:
  NavHint() : Node("navigation_hint") {
    sub_ = this->create_subscription<cpp_package::msg::ObstacleFeedback>(
        "/obstacle_feedback", 10,
        std::bind(&NavHint::cb, this, std::placeholders::_1));
    
    pub_ = this->create_publisher<cpp_package::msg::NavHint>("/nav_hint", 10);
    
    RCLCPP_INFO(this->get_logger(), "Navigation Hint Node started");
  }
  
private:
  void cb(const cpp_package::msg::ObstacleFeedback::SharedPtr m) {
    cpp_package::msg::NavHint h;
    h.header = m->header;

    /* 기본 룰:
       1) 정면 Danger → STOP
       2) 정면 Warn  → SLOW
       3) 좌/우 Danger 우선 회피
    */
    if (m->level_center == 2) {
      h.action = "STOP";
    }
    else if (m->level_center == 1) {
      h.action = "SLOW";
    }
    else {
      if (m->level_left > m->level_right) {
        h.action = "TURN_RIGHT";
      }
      else if (m->level_right > m->level_left) {
        h.action = "TURN_LEFT";
      }
      else {
        h.action = "GO";
      }
    }
    
    pub_->publish(h);
    
    RCLCPP_INFO(this->get_logger(), "Action: %s (L:%d C:%d R:%d)", 
               h.action.c_str(), m->level_left, m->level_center, m->level_right);
  }
  
  rclcpp::Subscription<cpp_package::msg::ObstacleFeedback>::SharedPtr sub_;
  rclcpp::Publisher<cpp_package::msg::NavHint>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavHint>());
  rclcpp::shutdown();
  return 0;
}
