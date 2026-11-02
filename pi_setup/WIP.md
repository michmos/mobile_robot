# Discovery server on the Pi: WIP

Status: not working. Topics do not show up on the Pi itself. Nodes do. Paused.

## Works
- `fastdds discovery -i 0 -l 0.0.0.0 -p 11811` runs as a systemd service.
- `ros2 node list` on the Pi shows the robot's nodes (with the daemon, after `ros2 daemon stop && ros2 daemon start`).
- The laptop worked with the server earlier (before the Pi-local problem).
- The robot's services get the right variables (`EnvironmentFile=/home/mypi/ros_env`).

## Does not work
- `ros2 topic list` (daemon or `--no-daemon`) shows only `/rosout` and `/parameter_events`. Sometimes the robot's topics appear, then disappear within seconds.
- A fresh `ros2 run demo_nodes_cpp talker` does not show `/chatter`.
- After a reboot, the ROS 2 daemon came back without topics even with `ROS_SUPER_CLIENT=true`.
- Server start at boot: systemd reports started at 17:08:25, the server logs "Server started" at 17:09:02.
- Server CPU: 23.7 s over about 9 minutes (~4% average). Higher than estimated.

## Likely cause
- The ROS 2 daemon is a plain discovery client, so it does not receive user topics (rmw_fastrtps #499, https://github.com/ros2/rmw_fastrtps/issues/499). Fast DDS docs say the daemon must be a Super Client: https://eprosima-fast-dds.readthedocs.io/en/latest/fastdds/ros2/discovery_server/ros2_discovery_server.html
- The Super Client XML from the 2.12 docs does not validate against Kilted's Fast DDS 3.2.5 schema (`/opt/ros/kilted/share/fastdds/fastdds_profiles.xsd`):
  - `SUPER_CLIENT` is not a valid `discoveryProtocol` value in 3.2.5 (valid: SIMPLE, CLIENT, SERVER, BACKUP, NONE).
  - `discoveryServersList` takes `<locator>` entries, not `<RemoteServer>` blocks.
- Unverified: whether `ROS_SUPER_CLIENT=true` (environment variable) alone makes the daemon a super client on 3.2.5. The Fast DDS env var docs describe it, but it did not hold across the reboot.

## Next steps
1. Write a 3.x profile (`discoveryProtocol` CLIENT, `discoveryServersList` with `<locator>`) and validate it with `xmllint --noout --schema /opt/ros/kilted/share/fastdds/fastdds_profiles.xsd <file>`.
2. Test on the Pi with `FASTRTPS_DEFAULT_PROFILES_FILE` set, and check `ros2 topic list` plus a talker/listener pair.
3. Check whether the server needs to start before the robot (boot took ~37 s to log "Server started").
4. Decide whether the discovery server is needed for Pi-local communication at all. Default multicast on one host may be enough.

## Pi state
- `/home/mypi/ros_env`: `ROS_SUPER_CLIENT=true`, no profile line.
- `/home/mypi/super_client.xml`: rejected by the parser. Not used.
