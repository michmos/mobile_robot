#!/bin/bash
# Started by mobile_robot.service. Gets the latest code of $BRANCH, rebuilds the
# workspace when the commit changed, flashes the ESP32 when the firmware changed
# and launches. A failed update, build or flash only logs a warning and the last
# installed build is launched, so the robot also starts offline.

BRANCH=main
REPO_URL=https://github.com/michmos/mobile_robot.git
ROS_SETUP=/opt/ros/kilted/setup.bash
WS=/home/mypi/robot_ws
REPO=$WS/src/mobile_robot
# commit of the last successful build, inside install/ so a removed install
# also forces a rebuild
BUILT_STAMP=$WS/install/.mobile_robot_commit
# git tree hash of esp32_firmware/ last flashed, outside install/ so a removed
# install does not force a reflash
FLASHED_STAMP=$WS/.esp32_firmware_tree
ESP32_PORT=/dev/esp32
ESP32_FQBN=esp32:esp32:esp32
# the first compile on the Pi takes several minutes
FLASH_TIMEOUT=1200

# never wait for credentials, and give up on a slow network instead of hanging
export GIT_TERMINAL_PROMPT=0
NET_TIMEOUT=60

warn() { echo "WARNING: $*" >&2; }

update_repo() {
  if [ ! -d "$REPO/.git" ]; then
    echo "Cloning $BRANCH of $REPO_URL into $REPO"
    timeout "$NET_TIMEOUT" git clone --branch "$BRANCH" --single-branch "$REPO_URL" "$REPO"
    return
  fi
  # don't overwrite changes made by hand on the Pi
  if ! git -C "$REPO" diff --quiet HEAD; then
    warn "uncommitted changes in $REPO, not updating (stash or discard them to resume updates)"
    return 0
  fi
  timeout "$NET_TIMEOUT" git -C "$REPO" fetch "$REPO_URL" "+refs/heads/$BRANCH:refs/remotes/origin/$BRANCH" &&
    git -C "$REPO" checkout -q -B "$BRANCH" "origin/$BRANCH"
}

build_ws() {
  local head
  head=$(git -C "$REPO" rev-parse HEAD) || return 1
  if [ -f "$WS/install/setup.bash" ] && [ "$(cat "$BUILT_STAMP" 2>/dev/null)" = "$head" ]; then
    echo "$head already built"
    return 0
  fi
  echo "Building $head"
  # subshell: the launch below must not see the build environment.
  # One package at a time, the Pi has little memory.
  (
    # shellcheck source=/dev/null
    source "$ROS_SETUP" && cd "$WS" && colcon build --parallel-workers 1
  ) || return 1
  echo "$head" > "$BUILT_STAMP"
}

flash_firmware() {
  local tree
  tree=$(git -C "$REPO" rev-parse HEAD:esp32_firmware) || return 1
  if [ "$(cat "$FLASHED_STAMP" 2>/dev/null)" = "$tree" ]; then
    echo "firmware $tree already flashed"
    return 0
  fi
  if ! command -v arduino-cli > /dev/null; then
    warn "arduino-cli not found"
    return 1
  fi
  if [ ! -e "$ESP32_PORT" ]; then
    warn "$ESP32_PORT not found"
    return 1
  fi
  echo "Flashing firmware $tree to $ESP32_PORT"
  timeout "$FLASH_TIMEOUT" arduino-cli compile --upload -p "$ESP32_PORT" \
    --fqbn "$ESP32_FQBN" "$REPO/esp32_firmware" || return 1
  echo "$tree" > "$FLASHED_STAMP"
}

update_repo || warn "updating $REPO failed, using the current checkout"
if [ -d "$REPO/.git" ]; then
  build_ws || warn "build failed, launching the last installed build"
  flash_firmware || warn "flashing the ESP32 failed, launching with the current firmware"
fi

# the setup file only exists on the Pi, nothing for shellcheck to follow
# shellcheck source=/dev/null
source "$WS/install/setup.bash"
exec ros2 launch mobile_robot mobile_robot_launch.py
