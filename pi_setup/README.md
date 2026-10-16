# Raspberry Pi setup
Files to start the robot automatically on boot. Run everything on the Pi.

| File | Install location | Purpose |
|------|------------------|---------|
| `udev.d/99-esp32.rules` | `/etc/udev/rules.d/` | Stable `/dev/esp32` name for the ESP32 (CH340, `1a86:7523`) |
| `start_robot.sh` | `/home/mypi/` | Updates, builds and launches the code, called from mobile_robot.service |
| `systemd/mobile_robot.service` | `/etc/systemd/system/` | Runs `/home/mypi/start_robot.sh` on boot, restarts on failure |

## Install
Follow these instructions to setup the automatic on boot launch - only needs to be done once
```bash
cd pi_setup
sudo cp udev.d/99-esp32.rules /etc/udev/rules.d/
sudo cp systemd/mobile_robot.service /etc/systemd/system/

sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/esp32                      # should point to ttyUSB0

sudo systemctl daemon-reload
sudo systemctl enable --now mobile_robot
```

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
```

The service gives up after 5 starts within 90 s (`StartLimit*` in the unit).

The launch also ends (and the service restarts it) when the ESP32 hardware component leaves the `active` state at runtime, e.g. after a failed handshake: the `hardware_watchdog` node exits then, see "Hardware watchdog" in `OVERVIEW.md`. `ros2 launch` exits with code 0 in that case, hence `Restart=always`.
