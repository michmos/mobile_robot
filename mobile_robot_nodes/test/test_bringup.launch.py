"""
Integration test: the whole stack with mock hardware, cmd_vel -> odom.

Starts mobile_robot_launch.py with use_mock_hardware:=true (no esp32, lidar
or foxglove) and checks that the nodes fit together: controllers come up,
joint states and the description are published, and a command on the
twist_mux input moves the odometry.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

# controller_manager loads the plugins and the spawners wait for it, this can
# take a few seconds on a loaded CI machine
STARTUP_TIMEOUT = 20.0
TIMEOUT = 5.0
WHEEL_JOINTS = {'left_wheel_joint', 'right_wheel_joint'}
CONTROLLERS = {'joint_state_broadcaster', 'mobile_robot_controller'}
# forward speed sent on the twist_mux input (m/s)
LINEAR_X = 0.2


@pytest.mark.launch_test
def generate_test_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobile_robot'),
            'launch', 'mobile_robot_launch.py')),
        launch_arguments={
            'use_mock_hardware': 'true',
            'use_lidar': 'false',
            'use_foxglove': 'false',
        }.items())
    return LaunchDescription([
        bringup,
        launch_testing.actions.ReadyToTest(),
    ])


class TestBringup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_bringup')
        self.odom = []
        self.node.create_subscription(
            Odometry, '/odom', self.odom.append, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.cmd_vel_pub = self.node.create_publisher(
            TwistStamped, '/cmd_vel_joy', 10)
        # twist_mux drops an input after its 0.5 s timeout, so keep sending
        self.linear_x = None
        self.node.create_timer(0.05, self.publish_cmd_vel)
        self.wait_controllers_active()

    def tearDown(self):
        self.tf_listener.unregister()
        self.node.destroy_node()

    def publish_cmd_vel(self):
        if self.linear_x is None:
            return
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.linear_x
        self.cmd_vel_pub.publish(msg)

    def spin_until(self, condition, timeout=TIMEOUT):
        """Spin until condition() is true, False after timeout seconds."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def active_controllers(self, client):
        future = client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        if future.result() is None:
            return set()
        return {c.name for c in future.result().controller
                if c.state == 'active'}

    def wait_controllers_active(self):
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=STARTUP_TIMEOUT),
                        'controller_manager not available')
        self.assertTrue(
            self.spin_until(
                lambda: CONTROLLERS <= self.active_controllers(client),
                STARTUP_TIMEOUT),
            f'controllers not active: {CONTROLLERS}')
        self.node.destroy_client(client)

    def lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target, source, rclpy.time.Time())
        except TransformException:
            return None

    def latest_odom(self):
        return self.odom[-1] if self.odom else None

    def drive_forward(self):
        """Command forward motion until odometry has moved, return start x."""
        # all tests share one launch, so the pose carries over from earlier
        # tests; only movement relative to the start of this test counts
        self.assertTrue(self.spin_until(lambda: self.odom), 'no /odom')
        start_x = self.latest_odom().pose.pose.position.x
        self.linear_x = LINEAR_X
        self.assertTrue(
            self.spin_until(
                lambda: self.latest_odom().pose.pose.position.x
                > start_x + 0.05),
            'odometry did not move forward')
        return start_x

    def test_controllers_are_active(self):
        # setUp already waits for this, the test makes the check explicit
        client = self.node.create_client(
            ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT))
        self.assertLessEqual(CONTROLLERS, self.active_controllers(client))

    def test_joint_states_publish_both_wheels(self):
        names = set()
        self.node.create_subscription(
            JointState, '/joint_states',
            lambda msg: names.update(msg.name), 10)

        self.assertTrue(self.spin_until(lambda: WHEEL_JOINTS <= names),
                        f'/joint_states has {names}')

    def test_robot_description_is_published(self):
        descriptions = []
        # robot_state_publisher latches the description
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(
            String, '/robot_description', descriptions.append, qos)

        self.assertTrue(self.spin_until(lambda: descriptions))
        for link in ('base_link', 'chassis', 'left_wheel', 'right_wheel'):
            self.assertIn(f'<link name="{link}"', descriptions[0].data)

    def test_tf_has_robot_links(self):
        # fixed joints come from /tf_static, the wheels need /joint_states
        for link in ('chassis', 'left_wheel', 'right_wheel', 'lidar'):
            self.assertTrue(
                self.spin_until(lambda: self.lookup('base_link', link)),
                f'no transform base_link -> {link}')

    def test_cmd_vel_moves_odometry_forward(self):
        self.drive_forward()

        odom = self.latest_odom()
        self.assertEqual(odom.header.frame_id, 'odom')
        self.assertEqual(odom.child_frame_id, 'base_link')
        # straight line: no sideways drift or turning
        self.assertAlmostEqual(odom.pose.pose.position.y, 0.0, delta=1e-3)
        self.assertAlmostEqual(odom.pose.pose.orientation.z, 0.0, delta=1e-3)
        self.assertTrue(
            self.spin_until(lambda: math.isclose(
                self.latest_odom().twist.twist.linear.x, LINEAR_X,
                abs_tol=0.02)),
            f'odom twist {self.latest_odom().twist.twist.linear.x}')
        self.assertAlmostEqual(
            self.latest_odom().twist.twist.angular.z, 0.0, delta=1e-3)

    def test_tf_has_odom_to_base_link_after_moving(self):
        start_x = self.drive_forward()

        def moved():
            t = self.lookup('odom', 'base_link')
            return t is not None and t.transform.translation.x > start_x + 0.05

        self.assertTrue(self.spin_until(moved),
                        'odom -> base_link did not follow the odometry')

    def test_robot_stops_when_commands_stop(self):
        self.drive_forward()

        self.linear_x = None
        # twist_mux stops forwarding the input after 0.5 s (without sending
        # zero), then the controller's cmd_vel_timeout (0.5 s) zeroes the
        # command (#40)
        self.assertTrue(
            self.spin_until(
                lambda: abs(self.latest_odom().twist.twist.linear.x) < 1e-3),
            'robot still moving at '
            f'{self.latest_odom().twist.twist.linear.x} m/s')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
