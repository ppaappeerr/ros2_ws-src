#!/usr/bin/env python3
# filepath: /home/p/ros2_ws/src/cpp_package/scripts/haptic_driver_node.py
import rclpy
import time
from rclpy.node import Node

# 라즈파이 GPIO 사용 시
try:
    from gpiozero import PWMOutputDevice
    GPIO_AVAILABLE = True
    PIN = 18   # BCM 핀 번호
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: gpiozero not available. Running in simulation mode.")

from cpp_package.msg import NavHint

class HapticDriver(Node):
    def __init__(self):
        super().__init__('haptic_driver')
        
        if GPIO_AVAILABLE:
            self.motor = PWMOutputDevice(PIN, frequency=200)
            self.get_logger().info(f"Haptic motor initialized on GPIO pin {PIN}")
        else:
            self.motor = None
            self.get_logger().info("Running in simulation mode (no GPIO)")
            
        self.create_subscription(NavHint, '/nav_hint', self.cb, 10)
        self.get_logger().info("Haptic Driver Node started")

    def cb(self, msg):
        act = msg.action
        self.get_logger().info(f"Received action: {act}")
        
        if act == 'STOP':
            self.buzz(1.0, 1.0)  # 강한 진동 1초
        elif act == 'SLOW':
            self.buzz(0.6, 0.5)  # 중간 진동 0.5초
        elif act == 'TURN_LEFT':
            self.buzz(0.8, 0.2)  # 짧은 강한 진동
        elif act == 'TURN_RIGHT':
            self.buzz(0.8, 0.2)  # 짧은 강한 진동
        else:  # GO
            if self.motor:
                self.motor.value = 0.0  # 진동 끄기
    
    def buzz(self, duty, dur):
        if self.motor:
            self.motor.value = duty
            time.sleep(dur)
            self.motor.value = 0.0
        else:
            # 시뮬레이션 모드
            print(f"🔊 BUZZ: duty={duty}, duration={dur}s")
            time.sleep(dur)

def main():
    rclpy.init()
    try:
        node = HapticDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()