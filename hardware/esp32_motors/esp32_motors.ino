#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// --- Pin Definitions (Adjust for your Motor Driver) ---
const int IN1 = 25; 
const int IN2 = 26;
const int IN3 = 27;
const int IN4 = 14;
const int ENA = 32; // PWM Pin for speed
const int ENB = 33; // PWM Pin for speed

// --- ROS Objects ---
rcl_subscription_t subscriber;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

// --- Motor Logic Functions ---
void move_robot(int speedA, int speedB, bool dir1, bool dir2, bool dir3, bool dir4) {
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
  digitalWrite(IN1, dir1);
  digitalWrite(IN2, dir2);
  digitalWrite(IN3, dir3);
  digitalWrite(IN4, dir4);
}

void stop_robot() { move_robot(0, 0, LOW, LOW, LOW, LOW); }

// --- ROS Callback ---
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String command = String(msg->data.data);

  if (command == "FORWARD") {
    move_robot(200, 200, HIGH, LOW, HIGH, LOW);
  } else if (command == "LEFT") {
    move_robot(150, 150, LOW, HIGH, HIGH, LOW);
  } else if (command == "RIGHT") {
    move_robot(150, 150, HIGH, LOW, LOW, HIGH);
  } else if (command == "ROTATE") {
    move_robot(130, 130, HIGH, LOW, LOW, HIGH); // Slower pivot
  } else {
    stop_robot();
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/esp32/motor_commands"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
