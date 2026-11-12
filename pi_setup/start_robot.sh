#!/bin/bash
# Started by mobile_robot.service. Gets the latest code of $BRANCH and the source
# dependencies pinned in its dependencies.repos, rebuilds the workspace when a
# checked out commit changed and launches. A failed update or build only
# logs a warning and the last installed build is launched, so the robot also
# starts offline.

BRANCH=main
REPO_URL=https://github.com/michmos/mobile_robot.git
ROS_SETUP=/opt/ros/kilted/setup.bash
WS=/home/mypi/robot_ws
REPO=$WS/src/mobile_robot
# commits of the src/ repos at the last successful build, inside install/ so a
# removed install also forces a rebuild
BUILT_STAMP=$WS/install/.src_commits

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

import_deps() {
  if ! command -v vcs >/dev/null; then
    warn "vcs not found (sudo apt install python3-vcstool), source dependencies not updated"
    return 0
  fi
  timeout "$NET_TIMEOUT" vcs import --input "$REPO/dependencies.repos" "$WS/src"
}

# "<dir> <commit>" for every git repo in src/
src_commits() {
  local dir head
  for dir in "$WS"/src/*/; do
    [ -e "$dir/.git" ] || continue
    head=$(git -C "$dir" rev-parse HEAD) || return 1
    echo "$(basename "$dir") $head"
  done
}

build_ws() {
  local commits
  commits=$(src_commits) || return 1
  if [ -f "$WS/install/setup.bash" ] && [ "$(cat "$BUILT_STAMP" 2>/dev/null)" = "$commits" ]; then
    echo "already built: $commits"
    return 0
  fi
  echo "Building: $commits"
  # subshell: the launch below must not see the build environment.
  # One package at a time, the Pi has little memory.
  (
    # shellcheck source=/dev/null
    source "$ROS_SETUP" && cd "$WS" && colcon build --parallel-workers 1 --cmake-args -DBUILD_TESTING=OFF
  ) || return 1
  echo "$commits" > "$BUILT_STAMP"
}

update_repo || warn "updating $REPO failed, using the current checkout"
if [ -d "$REPO/.git" ]; then
  import_deps || warn "importing source dependencies failed, using the current checkouts"
  build_ws || warn "build failed, launching the last installed build"
fi

# the setup file only exists on the Pi, nothing for shellcheck to follow
# shellcheck source=/dev/null
source "$WS/install/setup.bash"
exec ros2 launch mobile_robot mobile_robot_launch.py
