#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

const int LED_LEFT   = 21; 
const int LED_RIGHT  = 18; 
const int LED_STOP   = 2;  
const int LED_FWD    = 33;
const int LED_SLOW   = 4;  

rcl_subscription_t subscriber;
std_msgs__msg__String msg_sub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- MANDATORY STRING BUFFER ---
char test_buffer[20]; 

void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // Reset all LEDs
  digitalWrite(LED_LEFT, LOW); digitalWrite(LED_RIGHT, LOW);
  digitalWrite(LED_STOP, LOW); digitalWrite(LED_FWD, LOW);
  digitalWrite(LED_SLOW, LOW);

  // Use strstr on the pre-allocated buffer
  if (strstr(msg->data.data, "FORWARD"))      digitalWrite(LED_FWD, HIGH);
  else if (strstr(msg->data.data, "LEFT"))    digitalWrite(LED_LEFT, HIGH);
  else if (strstr(msg->data.data, "RIGHT"))   digitalWrite(LED_RIGHT, HIGH);
  else if (strstr(msg->data.data, "STOP"))    digitalWrite(LED_STOP, HIGH);
  else if (strstr(msg->data.data, "ROTATE")) digitalWrite(LED_SLOW, HIGH);
}

void setup() {
  // 1. PIN TEST (POST)
  int pins[] = {21, 18, 2, 33, 4};
  for(int i=0; i<5; i++) {
    pinMode(pins[i], OUTPUT);
    digitalWrite(pins[i], HIGH);
  }
  delay(1000); 
  for(int i=0; i<5; i++) digitalWrite(pins[i], LOW);

  set_microros_transports();
  delay(1000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "jime_esp32_serial", "", &support);

  // --- 2. CRITICAL FIX: PRE-ALLOCATE STRING MEMORY ---
  // Without this, the micro-ROS executor cannot "fill" the message.
  msg_sub.data.data = test_buffer;
  msg_sub.data.capacity = 20;

  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "robot_cmd");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
