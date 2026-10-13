
# Overview
[![CI](https://github.com/michmos/mobile_robot/actions/workflows/ci.yml/badge.svg?branch=dev)](https://github.com/michmos/mobile_robot/actions/workflows/ci.yml?query=branch%3Adev)

A differential-drive mobile robot split across two compute targets, connected over micro-ROS (serial transport).

This directory (`ros2_ws/src/mobile_robot/`) groups the packages below; it is not itself a ROS package (no top-level `package.xml`).

- **`esp32_firmware/`** — Arduino/micro-ROS firmware for the ESP32 (encoders, L298N motor driver). No `package.xml`, so colcon ignores it; built/flashed separately via Arduino IDE/arduino-cli.
- **`mobile_robot_hardware/`** — `ros2_control` hardware interface plugin; owns the serial link to the ESP32.
- **`mobile_robot_controller/`** — `ros2_control` controller plugin.
- **`mobile_robot_nodes/`** — the ROS package colcon actually builds (named `mobile_robot`): launch files, `controller_manager` config, URDF/xacro description.
- **`pi_setup/`** — boot-time systemd/udev setup for running the stack on the Pi, see `pi_setup/README.md`.
- **`docker/Dockerfile.ros2`** — ROS 2 kilted + arduino-cli image used by CI and the agent sandbox (not by the robot).

## CI
`.github/workflows/ci.yml` runs on pushes to `dev`/`main` and on pull requests:
- **`ros`**: builds `docker/Dockerfile.ros2`, then `colcon build` + `colcon test` for `mobile_robot_hardware`, `mobile_robot_controller` and `mobile_robot` inside it (with `xv_11_driver` cloned next to the repo), fails on `colcon test-result --verbose` and uploads the test results as the `test-results` artifact.
- **`firmware`**: `arduino-cli compile --fqbn esp32:esp32:esp32 esp32_firmware` with the core pinned to `esp32:esp32@2.0.17` (cached).
- **`shell`**: `shellcheck pi_setup/start_robot.sh`.

## Serial protocol (pi ↔ ESP32)
- Line based text protocol, defined in `esp32_firmware/inc/Messages.hpp`. The pi side lives in `mobile_robot_hardware/src/wire_protocol.cpp`; the config payload, its range check and the CRC are shared headers (`esp32_firmware/inc/ConfigPayload.hpp`, `Crc16.hpp`).
- **Firmware and hardware plugin must always be updated together**, the two sides reject each other's lines after a wire format change.
- Configuration handshake: the ESP32 sends `S,SETUP` every 500 ms until configured. The pi sends `C,<handshake_id>,...,<crc>` every 500 ms until it gets the matching `S,ACK,<handshake_id>` (a `S,NACK,...` is retried), and gives up after 10 s. Each handshake has a new, increasing id and a fixed payload. State machine: `mobile_robot_hardware/include/mobile_robot_hardware/handshake.hpp`.
- A handshake runs on `on_configure`, when the ESP32 restarts (SETUP while configured) and on a live parameter change. Until it is acked, `write()` sends zero velocity. A handshake that fails while active makes `read()` return `ERROR`: controller_manager deactivates the component and its controllers, and the process keeps running (no systemd restart).
- **Live tuning stops the robot briefly:** the hardware component's parameters (`pid_kp`, `pid_ki`, `pid_integral_limit`, `ff_slope_left/right`, `ff_min_duty_left/right`, defaults in `mobile_robot_nodes/description/ros2_control.xacro`) can be changed at runtime with `ros2 param set`. Every change starts a handshake: velocity commands are zero until the ACK, and the ESP32 brakes and resets its PI integrators when it applies the new values. Dragging e.g. an rqt slider therefore keeps the robot standing while tuning.
