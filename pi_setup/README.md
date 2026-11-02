# Raspberry Pi setup
Files to start the robot automatically on boot. Run everything on the Pi.

| File | Install location | Purpose |
|------|------------------|---------|
| `udev.d/99-esp32.rules` | `/etc/udev/rules.d/` | Stable `/dev/esp32` name for the ESP32 (CH340, `1a86:7523`) |
| `start_robot.sh` | `/home/mypi/` | Updates, builds and launches the code, called from mobile_robot.service |
| `systemd/mobile_robot.service` | `/etc/systemd/system/` | Runs `/home/mypi/start_robot.sh` on boot, restarts on failure |
| `systemd/ros_discovery_server.service` | `/etc/systemd/system/` | Runs the Fast DDS discovery server (ID 0, port 11811) on boot |
| `ros_env.example` | `/home/mypi/ros_env` | ROS environment shared by both services and interactive shells |

## Install
Follow these instructions to setup the automatic on boot launch - only needs to be done once
```bash
cd pi_setup
sudo cp udev.d/99-esp32.rules /etc/udev/rules.d/
sudo cp systemd/ros_discovery_server.service systemd/mobile_robot.service /etc/systemd/system/
cp ros_env.example /home/mypi/ros_env

sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/esp32                      # should point to ttyUSB0

sudo systemctl daemon-reload
sudo systemctl enable --now ros_discovery_server mobile_robot
```

Add this line to the shell rc of the Pi user (`~/.zshrc` for zsh, `~/.bashrc` for bash) so interactive shells get the same variables:
```zsh
set -a; . ~/ros_env; set +a
```

The Pi's nodes use `127.0.0.1` for the server, so `/home/mypi/ros_env` never needs editing.

The laptop needs `RMW_IMPLEMENTATION`, `ROS_DOMAIN_ID` and `ROS_SUPER_CLIENT=true` (same as the Pi), and `ROS_DISCOVERY_SERVER` set to the Pi's current IP, in its own shell config. The hotspot can give the Pi a new IP, so only the laptop's value changes then.

## Code updates on start
On every (re)start `start_robot.sh` gets the latest code before it launches:
- No `/home/mypi/robot_ws/src/mobile_robot`: clones the `dev` branch from GitHub.
- Otherwise: fetches `dev` and checks out `origin/dev`. Uncommitted changes in the checkout are left alone, the update is skipped with a warning.
- Runs `colcon build --parallel-workers 1` in `/home/mypi/robot_ws` only when the checked out commit differs from the last successful build (stored in `install/.mobile_robot_commit`) or no install exists.
- Offline, a git timeout (60 s) or a failed build only log a `WARNING` (`journalctl -u mobile_robot`) and the last installed build is launched. A failed build is retried on the next start.

The branch is `BRANCH=dev` at the top of the script, switch it to `main` once `main` is stable.

## Prerequisites
- `/home/mypi/start_robot.sh` exists, is executable and ends with `exec ros2 launch ...` so signals reach the launch
- ROS 2 kilted (`/opt/ros/kilted`), `git` and `colcon` are installed, the build dependencies are installed (rosdep)
- user `mypi` is in the `dialout` group
- `serial_port` in `mobile_robot_nodes/description/ros2_control.xacro` is `/dev/esp32`

## Usage

```bash
sudo systemctl stop mobile_robot          # before manual launches (serial port)
sudo systemctl restart mobile_robot
journalctl -u mobile_robot -f             # logs
journalctl -u mobile_robot -p err --since "10 min ago"  # errors only
sudo systemctl reset-failed mobile_robot  # after the restart limit was hit
journalctl -u ros_discovery_server -f     # discovery server logs
```

The service gives up after 5 starts within 90 s (`StartLimit*` in the unit).

The launch also ends (and the service restarts it) when the ESP32 hardware component leaves the `active` state at runtime, e.g. after a failed handshake: the `hardware_watchdog` node exits then, see "Hardware watchdog" in `OVERVIEW.md`. `ros2 launch` exits with code 0 in that case, hence `Restart=always`.
