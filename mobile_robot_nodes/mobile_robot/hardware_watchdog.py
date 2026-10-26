"""
Exit when a ros2_control hardware component leaves the 'active' state.

controller_manager takes a hardware component down when its read() or
write() fails (e.g. MobileRobotHardware after a failed esp32 handshake), but
ros2_control_node keeps running and has no option to exit on such a runtime
error. This node polls ~/list_hardware_components of the controller_manager
and exits with code 1 once a component it has seen 'active' is no longer
active. mobile_robot_launch.py shuts the launch down when it exits, and
systemd restarts the stack (pi_setup/systemd/mobile_robot.service), which
also runs the esp32 handshake from scratch.

A component that has not been active yet is not watched, so startup does not
trigger it (a failed start is covered by controller_manager's
hardware_components_initial_state.shutdown_on_initial_state_failure).
"""

from collections import namedtuple
import sys
import time

from controller_manager_msgs.srv import ListHardwareComponents
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

ACTIVE = 'active'
# label reported for a watched component that is no longer listed
MISSING = 'missing'
# a request without a response after this many seconds is dropped and sent
# again (a response can get lost right after discovery, see #52)
RESPONSE_TIMEOUT = 5.0

Update = namedtuple('Update', ['started', 'failed'])
Update.__doc__ = """Result of HardwareStateMonitor.update().

started: names of components that are watched from now on (newly active).
failed: {name: state label} of watched components that are not active.
"""


class HardwareStateMonitor:
    """Decides which hardware components have left the 'active' state."""

    def __init__(self):
        self._watched = set()

    @property
    def watched(self):
        """Names of the components that have been seen active."""
        return frozenset(self._watched)

    def update(self, states):
        """
        Take one list of component states, return what changed.

        :param states: {component name: lifecycle state label}, the full
            result of one list_hardware_components call
        :returns Update:
        """
        failed = {name: states.get(name, MISSING)
                  for name in self._watched
                  if states.get(name) != ACTIVE}
        started = {name for name, label in states.items()
                   if label == ACTIVE} - self._watched
        self._watched |= started
        return Update(started=started, failed=failed)


class HardwareWatchdog(Node):
    """Polls the hardware component states, see the module docstring."""

    def __init__(self):
        super().__init__('hardware_watchdog')
        controller_manager = self.declare_parameter(
            'controller_manager', '/controller_manager').value
        poll_period = self.declare_parameter('poll_period', 1.0).value
        if poll_period <= 0.0:
            raise ValueError(f'poll_period must be > 0, got {poll_period}')

        self.hardware_failed = False
        self._monitor = HardwareStateMonitor()
        self._client = self.create_client(
            ListHardwareComponents,
            f'{controller_manager}/list_hardware_components')
        self._pending = None
        self._sent_at = 0.0
        self._timer = self.create_timer(poll_period, self._poll)

    def _poll(self):
        if self._pending is not None:
            if time.monotonic() - self._sent_at < RESPONSE_TIMEOUT:
                return
            self.get_logger().warn(
                'No response from list_hardware_components, asking again')
            self._client.remove_pending_request(self._pending)
            self._pending = None
        # controller_manager not up yet (startup) or gone (its exit shuts
        # the launch down by itself)
        if not self._client.service_is_ready():
            return
        self._sent_at = time.monotonic()
        self._pending = self._client.call_async(
            ListHardwareComponents.Request())
        self._pending.add_done_callback(self._on_response)

    def _on_response(self, future):
        if future is not self._pending:
            return  # dropped after RESPONSE_TIMEOUT
        self._pending = None
        if future.cancelled() or future.result() is None:
            return
        update = self._monitor.update(
            {c.name: c.state.label for c in future.result().component})
        for name in sorted(update.started):
            self.get_logger().info(f"Watching hardware component '{name}'")
        if update.failed:
            for name, label in sorted(update.failed.items()):
                self.get_logger().error(
                    f"Hardware component '{name}' left the 'active' state "
                    f"(now '{label}'), exiting so the stack restarts")
            self.hardware_failed = True


def main(args=None):
    """Run the watchdog, return 1 after a hardware failure, else 0."""
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareWatchdog()
        while not node.hardware_failed:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # launch shutdown (SIGINT)
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
