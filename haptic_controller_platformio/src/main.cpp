#include <Arduino.h> // Required for PlatformIO

//
// micro-ROS Haptic Controller for Arduino Nano RP2040 Connect (8-Motor Array via WiFi)
//
// SUBSCRIBES TO: /safe_path_vector (geometry_msgs/msg/Vector3Stamped)
// CONTROLS: An array of 8 haptic motors to provide smooth, interpolated directional cues.
//

// ####################
// ## INCLUDES
// ####################
#include <WiFi.h> // Use WiFiNINA library for Nano RP2040 Connect
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/vector3_stamped.h>

// ####################
// ## NETWORK & ROS AGENT CONFIGURATION
// ####################

// --- WiFi Credentials ---
// Replace with your network details.
char ssid[] = "eGIST-WAP";

// Option 1: For a WiFi network WITH a password.
char pass[] = "";

// Option 2: For a WiFi network WITHOUT a password.
// Comment out the line above and uncomment the line below.
// char pass[] = "";

// --- micro-ROS Agent ---
// Replace with the IP address of the computer running the micro-ROS agent.
// This is typically your Raspberry Pi's IP address.
char agent_ip[] = "192.168.1.100";
size_t agent_port = 8888;


// ####################
// ## DEFINITIONS & CONFIGURATION
// ####################

// Haptic motor pins. This assumes a layout from LEFT to RIGHT.
const int NUM_MOTORS = 8;
const int motorPins[NUM_MOTORS] = {5, 6, 7, 8, 9, 10, 11, 12};

// Maximum PWM intensity for the motors.
const int MAX_INTENSITY = 255;

// micro-ROS settings
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ####################
// ## ROS 2 VARIABLES
// ####################
rcl_subscription_t subscriber;
geometry_msgs__msg__Vector3Stamped msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ####################
// ## HELPER FUNCTIONS
// ####################

void error_loop(){
  // Turn off all motors and blink the LED in case of an error
  for (int i = 0; i < NUM_MOTORS; i++) {
    digitalWrite(motorPins[i], LOW);
  }
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// Function to turn off all motors
void all_motors_off() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    analogWrite(motorPins[i], 0);
  }
}

// Callback function for the /safe_path_vector topic
void subscription_callback(const void * msgin)
{
  const geometry_msgs__msg__Vector3Stamped * msg = (const geometry_msgs__msg__Vector3Stamped *)msgin;

  // Calculate the angle from the vector, converting it to degrees (-180 to 180).
  float angle_rad = atan2(msg->vector.y, msg->vector.x);
  float angle_deg = angle_rad * 180.0 / M_PI;

  // --- Haptic Logic (8-Motor Interpolation) ---

  // 1. First, turn all motors off to clear the previous state.
  all_motors_off();

  // 2. Map the angle from [-90, 90] degrees to a floating-point index [0, 7] for our motors.
  // We clamp the angle to the [-90, 90] range to be safe.
  float clamped_angle_deg = constrain(angle_deg, -90.0, 90.0);
  float motor_index_float = map(clamped_angle_deg, -90.0, 90.0, 0.0, NUM_MOTORS - 1.0);

  // 3. Find the two motors to activate for interpolation.
  int motor_idx1 = floor(motor_index_float);
  int motor_idx2 = ceil(motor_index_float);

  // 4. Calculate the blending factor (0.0 to 1.0).
  // This determines the intensity distribution between the two motors.
  float blend = motor_index_float - motor_idx1;

  // 5. Calculate the intensity for each of the two motors.
  int intensity1 = (1.0 - blend) * MAX_INTENSITY;
  int intensity2 = blend * MAX_INTENSITY;

  // 6. Apply power to the motors.
  if (motor_idx1 == motor_idx2) {
    // The direction points directly at one motor.
    analogWrite(motorPins[motor_idx1], MAX_INTENSITY);
  } else {
    // The direction is between two motors.
    analogWrite(motorPins[motor_idx1], constrain(intensity1, 0, MAX_INTENSITY));
    analogWrite(motorPins[motor_idx2], constrain(intensity2, 0, MAX_INTENSITY));
  }
}


// ####################
// ## ARDUINO SETUP & LOOP
// ####################

void setup() {
  // Start serial for debugging purposes
  Serial.begin(115200);

  // Configure WiFi and micro-ROS transports
  // This function will connect to WiFi and then to the micro-ROS agent.
  set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
  
  // Configure all motor pins as outputs
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // LED on during setup
  delay(500);

  // --- micro-ROS Initialization ---
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "haptic_controller_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3Stamped),
    "/safe_path_vector"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  
  digitalWrite(LED_BUILTIN, LOW); // LED off indicates successful setup
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}