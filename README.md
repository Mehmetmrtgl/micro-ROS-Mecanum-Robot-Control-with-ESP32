## Mecanum Wheeled Robot Control (ESP32 + micro-ROS)

This project is a micro-ROS application running on ESP32 that enables controlling a Mecanum wheeled robot with ROS 2 infrastructure. The robot is directed via cmd_vel messages in the geometry_msgs/msg/Twist format.
## Features

    Holonomic movement support (forward, backward, lateral, diagonal, and rotation) with Mecanum wheel drive mechanics

    ROS 2 communication with micro-ROS

    Speed and direction control via cmd_vel

    Motor driving with PWM signal

    Full compatibility with ESP32

## Hardware Requirements

    ESP32 Development Board

    4 DC Motors (with Mecanum wheels)

    Motor Driver (PWM input)

    A computer running ROS 2 (e.g. Ubuntu, Jetson Orin, etc.)

## ROS 2 Configuration

ESP32 listens to the cmd_vel topic. The robot can be directed with the following sample message:

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"

    linear.x: Forward / Backward

    linear.y: Right / Left

    angular.z: Clockwise / Counterclockwise rotation

## Installation and Usage
  
    Upload this project to ESP32 via PlatformIO or Arduino IDE.
    
    Connect ESP32 to the computer running ROS 2 via USB.
    
    Start the micro-ROS agent from terminal:
    
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
    
    You can move the robot by sending cmd_vel commands.

## References

This project was developed based on the basic kinematic modeling from the following academic publication:

- T. M. R. Rodrigues, L. M. Gonçalves, and J. L. Afonso, *"Kinematic Model of a Mecanum Wheeled Mobile Robot,"* International Journal of Computer Applications, vol. 113, no. 3, 2015. [PDF link](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)
