#!/bin/bash

# the setup file only exists on the Pi, nothing for shellcheck to follow
# shellcheck source=/dev/null
source /home/mypi/robot_ws/install/setup.bash
exec ros2 launch mobile_robot mobile_robot_launch.py
