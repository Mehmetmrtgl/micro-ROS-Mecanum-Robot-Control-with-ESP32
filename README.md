# Mecanum Robot Control (ESP32 + micro-ROS)

[![CI](https://github.com/Mehmetmrtgl/micro-ROS-Mecanum-Robot-Control-with-ESP32/actions/workflows/ci.yml/badge.svg)](https://github.com/Mehmetmrtgl/micro-ROS-Mecanum-Robot-Control-with-ESP32/actions/workflows/ci.yml)
![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

ESP32 firmware that drives a four-wheel mecanum robot from ROS 2. It subscribes to `cmd_vel` over micro-ROS, converts the velocity command into four wheel speeds with mecanum inverse kinematics and drives the motors with PWM.

## Features

- Holonomic motion: forward/backward, sideways, diagonal and rotation in place
- `geometry_msgs/msg/Twist` on `/cmd_vel` over a serial micro-ROS link
- PWM speed and direction pin per motor (LEDC, 5 kHz, 8-bit)

## Hardware

- ESP32 DevKit v1
- Four DC motors with mecanum wheels
- Motor driver with one PWM and one direction input per motor
- A computer running ROS 2 Humble (Ubuntu, Jetson, ...)

Pins, set in `lib/motor_pins.h`:

| Motor | PWM | DIR |
|---|---|---|
| 1 | 32 | 33 |
| 2 | 14 | 27 |
| 3 | 21 | 19 |
| 4 | 5 | 18 |

Robot dimensions (wheel radius 0.04 m, half length 0.23 m, half width 0.15 m) are set in `lib/MecanumControl.h`.

## Kinematics

For a command (v<sub>x</sub>, v<sub>y</sub>, ω), wheel radius r and half dimensions L, W:

```text
w1 = (vx - vy - (L + W)·ω) / r
w2 = (vx + vy + (L + W)·ω) / r
w3 = (vx + vy - (L + W)·ω) / r
w4 = (vx - vy + (L + W)·ω) / r
```

The sign of each wheel speed sets the direction pin and its magnitude sets the PWM duty.

## Build and flash

Requires [PlatformIO](https://platformio.org/) (CLI or the VS Code extension).

```bash
git clone https://github.com/Mehmetmrtgl/micro-ROS-Mecanum-Robot-Control-with-ESP32.git
cd micro-ROS-Mecanum-Robot-Control-with-ESP32
pio run -t upload      # upload port is set to /dev/ttyUSB0 in platformio.ini
```

## Run

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# Example: forward at 0.5 m/s while turning at 0.3 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

| Field | Motion |
|---|---|
| `linear.x` | Forward / backward |
| `linear.y` | Left / right |
| `angular.z` | Rotation |

A joystick can drive the robot through [ros2-joystick](https://github.com/Mehmetmrtgl/ros2-joystick); wheel odometry comes from [mecanum-encoder-micro-ros](https://github.com/Mehmetmrtgl/mecanum-encoder-micro-ros) and [encoder_odometry](https://github.com/Mehmetmrtgl/encoder_odometry).

## Layout

```text
src/main.cpp           micro-ROS node and cmd_vel subscriber
lib/MecanumControl.h   Inverse kinematics and wheel commands
lib/motor_control.h    PWM and direction control of one motor
lib/motor_pins.h       Pin definitions
platformio.ini         Board, micro-ROS transport, serial port
```

## Reference

T. M. R. Rodrigues, L. M. Gonçalves and J. L. Afonso, "Kinematic Model of a Mecanum Wheeled Mobile Robot", *International Journal of Computer Applications*, vol. 113, no. 3, 2015. [PDF](https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf)

## License

MIT
