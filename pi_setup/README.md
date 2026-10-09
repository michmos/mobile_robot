# Raspberry Pi setup
Files to start the robot automatically on boot. Run everything on the Pi.

| File | Install location | Purpose |
|------|------------------|---------|
| `udev.d/99-esp32.rules` | `/etc/udev/rules.d/` | Stable `/dev/esp32` name for the ESP32 (CH340, `1a86:7523`) |
| `start_robot.sh` | `/home/mypi/` | Launch script, called from mobile_robot.service |
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

## Prerequisites
- `/home/mypi/start_robot.sh` exists, is executable and ends with `exec ros2 launch ...` so signals reach the launch
- user `mypi` is in the `dialout` group
- `serial_port` in `mobile_robot_nodes/description/ros2_control.xacro` is `/dev/esp32`

## Usage

```bash
sudo systemctl stop mobile_robot          # before manual launches (serial port)
sudo systemctl restart mobile_robot
journalctl -u mobile_robot -f             # logs
sudo systemctl reset-failed mobile_robot  # after the restart limit was hit
```

The service gives up after 5 starts within 90 s (`StartLimit*` in the unit).
