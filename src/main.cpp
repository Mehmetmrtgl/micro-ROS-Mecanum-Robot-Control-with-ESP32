
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "motor_control.h"
#include "motor_pins.h"
#include "MecanumControl.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif


MecanumControl mecanumRobot(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B,
  MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B,
  MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B,
  MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

rcl_subscription_t subscriber;

geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 msg_linear_velocity_x;
std_msgs__msg__Float32 msg_linear_velocity_y;
std_msgs__msg__Float32 msg_angular_velocity_z;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  msg_linear_velocity_x.data = msg->linear.x;
  msg_linear_velocity_y.data = msg->linear.y;
  msg_angular_velocity_z.data = msg->angular.z;
  
  mecanumRobot.moveWithCmdVel(msg_linear_velocity_x.data, msg_linear_velocity_y.data, msg_angular_velocity_z.data);
}

void setup() {
  Serial.begin(115200);
  
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "cmd_vel_subscriber_rclc", "", &support));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK(rclc_subscription_init_default(
    &subscriber, 
    &node, 
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), 
    "cmd_vel"));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));

  msg_linear_velocity_x.data = 0.0;
	msg_linear_velocity_y.data = 0.0;
  msg_angular_velocity_z.data = 0.0;    
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
