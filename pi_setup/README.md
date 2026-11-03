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
- No `/home/mypi/robot_ws/src/mobile_robot`: clones the `main` branch from GitHub.
- Otherwise: fetches `main` and checks out `origin/main`. Uncommitted changes in the checkout are left alone, the update is skipped with a warning.
- Runs `colcon build --parallel-workers 1` in `/home/mypi/robot_ws` only when the checked out commit differs from the last successful build (stored in `install/.mobile_robot_commit`) or no install exists.
- Flashes the ESP32 with `arduino-cli compile --upload -p /dev/esp32 --fqbn esp32:esp32:esp32` only when `esp32_firmware/` changed: its git tree hash (`git rev-parse HEAD:esp32_firmware`) differs from the last successful flash, stored in `/home/mypi/robot_ws/.esp32_firmware_tree` (outside `install/`, so a removed install does not reflash). Changes outside `esp32_firmware/` never reflash. Compile and upload are limited to 20 min; the first compile on the Pi can take several minutes. Delete the stamp file to force a reflash.
- Offline, a git timeout (60 s), a failed build or a failed flash (also missing `arduino-cli` or `/dev/esp32`) only log a `WARNING` (`journalctl -u mobile_robot`) and the last installed build is launched with the firmware currently on the ESP32. A failed build or flash is retried on the next start.

The branch is `BRANCH=main` at the top of the script.

## Prerequisites
- `/home/mypi/start_robot.sh` exists, is executable and ends with `exec ros2 launch ...` so signals reach the launch
- ROS 2 kilted (`/opt/ros/kilted`), `git` and `colcon` are installed, the build dependencies are installed (rosdep)
- user `mypi` is in the `dialout` group
- `serial_port` in `mobile_robot_nodes/description/ros2_control.xacro` is `/dev/esp32`
- arduino-cli, the `esp32:esp32` core and the `ESP32Encoder` library for the firmware flash, see below

### Firmware flash prerequisites
Install the exact versions pinned in `docker/Dockerfile.ros2` (`ARDUINO_CLI_VERSION`, `ESP32_CORE_VERSION`, `ESP32ENCODER_VERSION`), so the Pi flashes what CI compile-checks. Run as `mypi`:
```bash
DOCKERFILE=/home/mypi/robot_ws/src/mobile_robot/docker/Dockerfile.ros2
pin() { sed -n "s/^ARG $1=//p" "$DOCKERFILE"; }

# arduino-cli (the install script picks the arm64 build on the Pi)
curl -fsSL "https://raw.githubusercontent.com/arduino/arduino-cli/$(pin ARDUINO_CLI_VERSION)/install.sh" \
  | sudo BINDIR=/usr/local/bin sh -s "$(pin ARDUINO_CLI_VERSION)"

arduino-cli config init --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install "esp32:esp32@$(pin ESP32_CORE_VERSION)"
arduino-cli lib update-index
arduino-cli lib install "ESP32Encoder@$(pin ESP32ENCODER_VERSION)"
sudo apt install python3-serial   # used by the core's post-build step
```

When a version in the Dockerfile changes, install the new one on the Pi the same way (`core install`/`lib install` with the new version; `core uninstall`/`lib uninstall` the old one).

Check once by hand that the upload works without pressing BOOT, i.e. the CH340's DTR/RTS lines reset the ESP32 into the bootloader:
```bash
sudo systemctl stop mobile_robot
arduino-cli compile --upload -p /dev/esp32 --fqbn esp32:esp32:esp32 /home/mypi/robot_ws/src/mobile_robot/esp32_firmware
```
If it hangs at `Connecting...`, automatic flashing will not work on this board.

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
