#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include "cpp_package/msg/nav_hint.hpp"

class HapticDriver : public rclcpp::Node {
public:
    HapticDriver() : Node("haptic_driver") {
        // GPIO 핀 설정 (시뮬레이션 모드)
        gpio_pin_ = 18;
        
        // 구독자 생성
        sub_ = this->create_subscription<cpp_package::msg::NavHint>(
            "/nav_hint", 10,
            std::bind(&HapticDriver::navHintCallback, this, std::placeholders::_1));
        
        // GPIO 초기화 시도
        initGPIO();
        
        RCLCPP_INFO(this->get_logger(), "Haptic Driver Node started");
    }
    
    ~HapticDriver() {
        cleanup();
    }

private:
    void initGPIO() {
        // GPIO 핀 초기화 (라즈파이에서만 동작)
        std::string export_cmd = "echo " + std::to_string(gpio_pin_) + " > /sys/class/gpio/export 2>/dev/null";
        std::string direction_cmd = "echo out > /sys/class/gpio/gpio" + std::to_string(gpio_pin_) + "/direction 2>/dev/null";
        
        system(export_cmd.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        system(direction_cmd.c_str());
        
        gpio_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "GPIO pin %d initialized", gpio_pin_);
    }
    
    void cleanup() {
        if (gpio_initialized_) {
            setGPIO(false);
            std::string unexport_cmd = "echo " + std::to_string(gpio_pin_) + " > /sys/class/gpio/unexport 2>/dev/null";
            system(unexport_cmd.c_str());
        }
    }
    
    void setGPIO(bool value) {
        if (!gpio_initialized_) {
            // 시뮬레이션 모드
            RCLCPP_INFO(this->get_logger(), "🔊 GPIO %d: %s", gpio_pin_, value ? "ON" : "OFF");
            return;
        }
        
        // 실제 GPIO 제어
        std::string gpio_path = "/sys/class/gpio/gpio" + std::to_string(gpio_pin_) + "/value";
        std::ofstream gpio_file(gpio_path);
        if (gpio_file.is_open()) {
            gpio_file << (value ? "1" : "0");
            gpio_file.close();
        }
    }
    
    void buzz(int duration_ms, int pattern = 1) {
        switch (pattern) {
            case 1: // 연속 진동
                setGPIO(true);
                std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
                setGPIO(false);
                break;
                
            case 2: // 펄스 진동
                for (int i = 0; i < duration_ms / 200; i++) {
                    setGPIO(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    setGPIO(false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
                break;
                
            case 3: // 빠른 펄스
                for (int i = 0; i < duration_ms / 100; i++) {
                    setGPIO(true);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    setGPIO(false);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                break;
        }
    }
    
    void navHintCallback(const cpp_package::msg::NavHint::SharedPtr msg) {
        std::string action = msg->action;
        
        RCLCPP_INFO(this->get_logger(), "Received action: %s", action.c_str());
        
        if (action == "STOP") {
            buzz(1000, 1);  // 1초 연속 진동
        }
        else if (action == "SLOW") {
            buzz(500, 2);   // 0.5초 펄스 진동
        }
        else if (action == "TURN_LEFT") {
            buzz(300, 3);   // 0.3초 빠른 펄스
        }
        else if (action == "TURN_RIGHT") {
            buzz(300, 3);   // 0.3초 빠른 펄스
        }
        else if (action == "GO") {
            setGPIO(false); // 진동 끄기
        }
    }
    
    rclcpp::Subscription<cpp_package::msg::NavHint>::SharedPtr sub_;
    int gpio_pin_;
    bool gpio_initialized_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HapticDriver>());
    rclcpp::shutdown();
    return 0;
}