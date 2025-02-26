# Mecanum Robot Motor Control with Micro-ROS

This project implements motor control for a Mecanum-wheeled robot using an ESP32 running micro-ROS and PlatformIO. The ESP32 subscribes to the `/cmd_vel` topic and adjusts the robot's movement accordingly.

## Features
- Uses **micro-ROS** to communicate with ROS2.
- Subscribes to **/cmd_vel** topic to receive velocity commands.
- Controls a **Mecanum-wheeled robot** using a motor driver.
- Supports **forward, backward, rotation, and diagonal movement** based on velocity commands.

## Hardware Requirements
- **ESP32** (with micro-ROS support)
- **Mecanum wheels and motors**
- **Motor driver**
- **Power supply**

## Software Requirements
- **PlatformIO** (for development)
- **Micro-ROS** (installed on ESP32)
- **ROS 2** (on a Jetson Orin or other ROS 2-compatible system)

## Installation & Setup
1. Clone this repository:
   ```sh
   git clone https://github.com/your-username/your-repo.git
   cd your-repo
   ```
2. Install **PlatformIO** if not already installed.
3. Open the project in **VS Code** with the PlatformIO extension.
4. Flash the code to your ESP32.
5. Ensure that your ESP32 is properly communicating with your ROS 2 system.

## Usage
- Run **ROS 2** on your host system.
- You can control the robot using a **joystick** by utilizing the repository: [ros2_joystick](https://github.com/Mehmetmrtgl/ros2_joystick.git).
- Alternatively, you can use the keyboard by running the following command:
  ```sh
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
- The robot should move accordingly based on the received command.


The robot should move accordingly based on the received command.
## Code Overview
- `main.cpp`: Implements micro-ROS subscriber logic and motor control.
- `motor_control.h/.cpp`: Contains motor control functions for Mecanum wheels.
- `motor_pins.h`: Defines motor driver pin connections.



