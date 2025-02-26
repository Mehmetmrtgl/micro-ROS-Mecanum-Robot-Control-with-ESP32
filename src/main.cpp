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

float linear_velocity_x = 0.0;
float angular_velocity_z = 0.0;

MecanumControl mecanumRobot(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B,
  MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B,
  MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B,
  MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);


rcl_subscription_t subscriber;
rcl_publisher_t encoder_publisher;


geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Float32 msg_linear_velocity;
std_msgs__msg__Float32 msg_angular_velocity;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}


void setMotorsBasedOnVelocity() {
  if (abs(linear_velocity_x) < 0.1) {
    linear_velocity_x = 0.0;  
  }

  if (abs(angular_velocity_z) < 0.1) {
    angular_velocity_z = 0.0;  
  }
  mecanumRobot.setSpeed(abs(linear_velocity_x) * 255); 

  if (linear_velocity_x > 0) {
    if (angular_velocity_z > 0) {
        mecanumRobot.moveLeftForward(); // sol çapraz ileri
    } else if (angular_velocity_z < 0) {
        mecanumRobot.moveRightForward();  // sağ çapraz ileri
    } else {
        mecanumRobot.moveForward();     // ileri hareket et
    }
  } else if (linear_velocity_x < 0) {
      if (angular_velocity_z > 0) {
          mecanumRobot.moveLeftBackward(); // sol çapraz geri
      } else if (angular_velocity_z < 0) {
          mecanumRobot.moveRightBackward();  // sağ çapraz geri
      } else {
          mecanumRobot.moveBackward();    // geriye hareket et
      }
  } else {
    mecanumRobot.setSpeed(abs(angular_velocity_z) * 255); 
      if (angular_velocity_z > 0) {
          mecanumRobot.rotateRight();     // sağ dönüş
      } else if (angular_velocity_z < 0) {
          mecanumRobot.rotateLeft();    // sol dönüş
      } else {
          mecanumRobot.stop();          // Dur
      }
  }
}

void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  linear_velocity_x = msg->linear.x;  
  angular_velocity_z = msg->angular.z;
  setMotorsBasedOnVelocity();
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

  msg_linear_velocity.data = 0.0;  
  msg_angular_velocity.data = 0.0;    

}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
