
# Overview
A differential-drive mobile robot split across two compute targets:

- **ESP32 microcontroller** (`esp32_firmware/`) — reads wheel encoders, drives motors via an L298N driver, talks to the Raspberry Pi over micro-ROS (serial transport).
- **Raspberry Pi** (`mobile_robot_nodes/`, a ROS 2 `ament_cmake` package named `mobile_robot`) — runs perception, sensor fusion, collision avoidance, and motor control as separate rclcpp nodes.

This repo is `ros2_ws/src/mobile_robot/` inside a colcon workspace — it is not itself a ROS package (no top-level `package.xml`); `mobile_robot_nodes/` is the actual ROS package colcon discovers. `esp32_firmware/` has no `package.xml`/`setup.py`, so colcon ignores it; it's built/flashed separately via the Arduino toolchain (it uses `#include "inc/..."` and `.ino`/`.hpp` conventions, i.e. Arduino IDE or arduino-cli, not PlatformIO).
