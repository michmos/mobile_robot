"""
Integration test: the launch exits when the hardware leaves 'active' (#2).

Runs `ros2 launch mobile_robot mobile_robot_launch.py` with mock hardware as
a child process, like systemd does on the robot (pi_setup/start_robot.sh), so
the hardware_watchdog's Shutdown ends only that child launch and the test can
check that it exits on its own. The test then takes the hardware component
out of 'active' through controller_manager, as controller_manager does after
a failed read() (esp32 handshake timeout).

Startup not triggering the watchdog is checked here and, together with normal
driving, by test_bringup.launch.py (the watchdog runs there too and would end
that launch early).
"""

import time
import unittest

from controller_manager_msgs.srv import (ListHardwareComponents,
                                         SetHardwareComponentState)
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.events.process import ProcessExited
import launch_testing
import launch_testing.actions
import launch_testing.markers
from lifecycle_msgs.msg import State
import pytest
import rclpy

# controller_manager loads the plugins first, this can take a few seconds on a
# loaded CI machine
STARTUP_TIMEOUT = 30.0
# watchdog poll period (1 s) plus the launch shutting all nodes down
SHUTDOWN_TIMEOUT = 20.0
TIMEOUT = 5.0
HW = 'mobile_robot_ros2_control'


# keep_alive: the outer launch must not end when the child launch exits, the
# test checks that exit itself
@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'mobile_robot', 'mobile_robot_launch.py',
             'use_mock_hardware:=true', 'use_lidar:=false',
             'use_foxglove:=false'],
        # line by line, not block buffered, so the output can be awaited
        additional_env={'PYTHONUNBUFFERED': '1'},
        output='screen',
    )
    return LaunchDescription([
        robot_launch,
        launch_testing.actions.ReadyToTest(),
    ]), {'robot_launch': robot_launch}


class TestHardwareWatchdog(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_hardware_watchdog')

    def tearDown(self):
        self.node.destroy_node()

    def call(self, srv_type, name, request):
        client = self.node.create_client(srv_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT),
                        f'{name} not available')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future,
                                         timeout_sec=TIMEOUT)
        self.node.destroy_client(client)
        self.assertIsNotNone(future.result(), f'no response from {name}')
        return future.result()

    def hardware_state(self):
        response = self.call(ListHardwareComponents,
                             '/controller_manager/list_hardware_components',
                             ListHardwareComponents.Request())
        return {c.name: c.state.label for c in response.component}.get(HW)

    def test_hardware_leaving_active_ends_the_launch(
            self, proc_info, proc_output, robot_launch):
        # the watchdog only watches a component it has seen active, so after
        # this line startup is over without having triggered it
        # ros2 launch prints its nodes' output on stdout
        proc_output.assertWaitFor(
            f"Watching hardware component '{HW}'", process=robot_launch,
            stream='stdout', timeout=STARTUP_TIMEOUT)
        # proc_info holds the latest start/exit event of each process
        self.assertNotIsInstance(proc_info[robot_launch], ProcessExited,
                                 'launch exited during startup')
        self.assertEqual(self.hardware_state(), 'active')

        start = time.monotonic()
        request = SetHardwareComponentState.Request()
        request.name = HW
        request.target_state = State(id=State.PRIMARY_STATE_INACTIVE,
                                     label='inactive')
        response = self.call(
            SetHardwareComponentState,
            '/controller_manager/set_hardware_component_state', request)
        self.assertTrue(response.ok, f'state is {response.state.label}')

        proc_info.assertWaitForShutdown(process=robot_launch,
                                        timeout=SHUTDOWN_TIMEOUT)
        proc_output.assertWaitFor(
            f"Hardware component '{HW}' left the 'active' state",
            process=robot_launch, stream='stdout', timeout=TIMEOUT)
        print(f'launch exited {time.monotonic() - start:.1f} s after the '
              f'deactivation with code {proc_info[robot_launch].returncode}')
