# shellcheck shell=bash
# Laptop side: points ROS 2 in the current shell at the discovery server on the Pi.
# Source it (bash or zsh) after sourcing ROS:  source robot_env.sh <pi-ip>
# Only this shell is affected, other ROS work on the laptop keeps its defaults.

if [ -z "$1" ]; then
  echo "usage: source robot_env.sh <pi-ip>" >&2
  return 1 2>/dev/null || exit 1
fi

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER="$1:11811"
# without it the ros2 CLI lists nodes but no topics
export ROS_SUPER_CLIENT=true

# the daemon keeps the discovery settings it was started with
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1
fi
echo "ROS_DISCOVERY_SERVER=$ROS_DISCOVERY_SERVER"
