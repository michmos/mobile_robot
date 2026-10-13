# CLAUDE.md
* This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
* This file is only meant for tuning claudes behaviour, not to store information. Everything else belongs in a different file - check ./ for other files such as TODO.md and OVERVIEW.md
* After doing a bigger change update @OVERVIEW.md

## Build and run
Build from the colcon workspace root (one level above this repo, see `ros2_ws/AGENTS.md`) 

Launch:
```bash
ros2 launch mobile_robot mobile_robot_launch.py
```

Tests: read `TESTING.md` before writing any test (standards, faking hardware, definition of done). `mobile_robot_hardware` has gtest unit tests for the esp32 handshake and the serial wire format, and component tests that drive the plugin against a fake esp32 on a pty (`test/fake_esp32.hpp`, fixture in `test/hardware_fixture.hpp`): its lifecycle and handshake (`test_mobile_robot_hardware`) and how incoming lines become joint states (`test_encoder_state`), all in `mobile_robot_hardware/test/`. Run them from the workspace root:
```bash
colcon test --packages-select mobile_robot_hardware && colcon test-result --verbose
```

ESP32 firmware (`esp32_firmware/esp32_firmware.ino`) is flashed independently through the Arduino IDE/arduino-cli with the `micro_ros_arduino` library installed; it is not part of the colcon build.

## Code Conventions
- Follow ROS 2 conventions throughout (topic/node naming, message usage, package layout). If you spot code that doesn't follow them, flag/correct it rather than silently matching it.
- Pi-side node classes use camelCase for members, trailing underscore for private variables, e.g. `dutyCycle_`, and lambda callbacks stored as `sub_`/`pub_`/`timer_`; keep new nodes consistent with this pattern.
- ESP32 firmware peripherals are wrapped in small non-copyable classes under `esp32_firmware/inc/` (`L298N`, `Encoder`) — copy/assignment is explicitly deleted since they own hardware pins.
- URDF is built with xacro macros: `mobile_robot_nodes/description/mobile_robot.urdf.xacro` includes `properties.xacro` (dimensions/masses), `materials.xacro`, and `intertial_macros.xacro` (inertia tensor helpers). Add new physical properties to `properties.xacro` rather than inlining constants in the URDF.
