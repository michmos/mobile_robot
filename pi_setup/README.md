# Raspberry Pi setup
Files to start the robot automatically on boot. Run everything on the Pi.

| File | Install location | Purpose |
|------|------------------|---------|
| `udev.d/99-esp32.rules` | `/etc/udev/rules.d/` | Stable `/dev/esp32` name for the ESP32 (CH340, `1a86:7523`) |
| `start_robot.sh` | `/home/mypi/` | Updates, builds and launches the code, called from mobile_robot.service |
| `systemd/mobile_robot.service` | `/etc/systemd/system/` | Runs `/home/mypi/start_robot.sh` on boot, restarts on failure |
| `systemd/ros_discovery_server.service` | `/etc/systemd/system/` | Runs the Fast DDS discovery server (UDP port 11811) on boot |
| `ros_env.example` | `/home/mypi/ros_env` | ROS environment shared by both services and interactive shells |
| `robot_env.sh` | laptop, anywhere | Sourced on the laptop to reach the Pi's discovery server |

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
set -a; . ~/ros_env; set +a; export ROS_SUPER_CLIENT=true
```
`ROS_SUPER_CLIENT` is for shells only: without it the `ros2` CLI lists nodes but no topics. The robot's nodes don't need it.

The Pi's nodes use `127.0.0.1` for the server, so `/home/mypi/ros_env` never needs editing.

## Discovery from the laptop
Multicast discovery usually fails on a phone hotspot, so all ROS processes register with the Fast DDS discovery server on the Pi. Only the laptop needs an address: the Pi's current IP (shown in the phone's hotspot settings or by `hostname -I` on the Pi).

On the laptop, after sourcing ROS:
```bash
source pi_setup/robot_env.sh <pi-ip>
ros2 topic list
```
It only changes the current shell. The laptop's firewall must accept UDP from the hotspot subnet (DDS sends data directly between the machines), e.g. `sudo ufw allow from <subnet>/24 proto udp`, or disable it while testing.

Things to know:
- The `ros2` daemon keeps the discovery settings it was started with. Run `ros2 daemon stop` after any change to the ROS variables (`robot_env.sh` does it).
- `ros2 topic list --no-daemon` only listens for 0.5 s by default, too short with a server. Use `--spin-time 5`.
- Fast DDS takes the machine's IP addresses when a process starts. If the Pi boots before it is on the hotspot, or the hotspot hands out a new IP (Android picks a new subnet per hotspot session), the laptop sees nothing or gets no data. Turn the hotspot on before booting the Pi, otherwise restart both services once the Pi is connected:
  ```bash
  sudo systemctl restart ros_discovery_server mobile_robot
  ```

### Hardware test checklist
1. Laptop: `ping <pi-ip>` must answer, otherwise the hotspot isolates its clients. Check the firewall rule above.
2. Pi: `journalctl -u ros_discovery_server -b` shows "Server started"; `ip -4 a show wlan0` shows the hotspot IP and the services started after it was assigned (`journalctl -b -u mobile_robot`).
3. Pi: `ros2 daemon stop; ros2 topic list --no-daemon --spin-time 5`, then `ros2 topic list` (with daemon).
4. Laptop: `source pi_setup/robot_env.sh <pi-ip>; ros2 topic list --no-daemon --spin-time 5; ros2 topic echo --once /<robot-topic>`. Check the other direction too: `ros2 topic pub /test std_msgs/msg/String "{data: hi}"` on the laptop, `ros2 topic echo /test std_msgs/msg/String` on the Pi.
5. Restart the hotspot (new IP), restart both services on the Pi and repeat 1-4 with the new IP.

If topics come and go, check whether the robot restarts: `systemctl show mobile_robot -p NRestarts`.

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
