
# Overview
A differential-drive mobile robot split across two compute targets, connected over micro-ROS (serial transport).

This directory (`ros2_ws/src/mobile_robot/`) groups the packages below; it is not itself a ROS package (no top-level `package.xml`).

- **`esp32_firmware/`** — Arduino/micro-ROS firmware for the ESP32 (encoders, L298N motor driver). No `package.xml`, so colcon ignores it; built/flashed separately via Arduino IDE/arduino-cli.
- **`mobile_robot_hardware/`** — `ros2_control` hardware interface plugin; owns the serial link to the ESP32.
- **`mobile_robot_controller/`** — `ros2_control` controller plugin.
- **`mobile_robot_nodes/`** — the ROS package colcon actually builds (named `mobile_robot`): launch files, `controller_manager` config, URDF/xacro description.
- **`pi_setup/`** — boot-time systemd/udev setup for running the stack on the Pi, see `pi_setup/README.md`.
