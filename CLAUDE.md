# CLAUDE.md
* This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
* This file is only meant for tuning claudes behaviour, not to store information. Everything else belongs in a different file - check /. for other files such as TODO.md and OVERVIEW.md

## Build and run
Build from the colcon workspace root (one level above this repo, i.e. `ros2_ws/`):

```bash
colcon build --packages-select mobile_robot
source install/setup.bash
```

Run the real robot stack (controller_manager against MobileRobotHardware, which talks to the ESP32 directly over its own serial protocol, with mobile_robot_controller spawned):

```bash
ros2 launch mobile_robot mobile_robot_launch.py
```

There is no test suite beyond the `ament_lint_auto`/`ament_lint_common` test_depends declared in `package.xml` (standard ROS 2 ament lint checks, run via `colcon test` from the workspace root).

ESP32 firmware (`esp32_firmware/esp32_firmware.ino`) is flashed independently through the Arduino IDE/arduino-cli with the `micro_ros_arduino` library installed; it is not part of the colcon build.

## Code Conventions
- Follow ROS 2 conventions throughout (topic/node naming, message usage, package layout). If you spot code that doesn't follow them, flag/correct it rather than silently matching it.
- Pi-side node classes use camelCase for members, trailing underscore for private variables, e.g. `dutyCycle_`, and lambda callbacks stored as `sub_`/`pub_`/`timer_`; keep new nodes consistent with this pattern (see `mobile_robot_controller.cpp`/`.hpp` for a current example - the original example nodes this convention was written against, `motor_control.cpp`/`perception.cpp`/`collision_avoid.cpp`, were removed as dead code once ros2_control replaced them).
- ESP32 firmware peripherals are wrapped in small non-copyable classes under `esp32_firmware/inc/` (`L298N`, `Encoder`) — copy/assignment is explicitly deleted since they own hardware pins.
- URDF is built with xacro macros: `mobile_robot_nodes/description/mobile_robot.urdf.xacro` includes `properties.xacro` (dimensions/masses), `materials.xacro`, and `intertial_macros.xacro` (inertia tensor helpers). Add new physical properties to `properties.xacro` rather than inlining constants in the URDF.

- Try to split up code into functions where it makes sense -> Try to avoid functions longer than 50 lines
- avoid code duplication - instead try to outsource the snippet into a separate function / object

## Chat conventions
* when using abbreviations i might not know, put the full term behind in () including the category e.g. `ff (feedforward - control theory)`
