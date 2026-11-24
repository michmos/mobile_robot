"""
Integration test: Nav2 on the whole stack while mapping.

Starts mobile_robot_launch.py with mock hardware, slam:=mapping and nav:=true,
and plays the XV-11: a fake /scan of a rectangular room, computed from the
lidar's current pose in odom (mock odometry is exact, so odom is the world).
Checks that every Nav2 server becomes active and that a NavigateToPose goal
0.5 m ahead drives the robot there through /cmd_vel_nav and twist_mux.

Takes ~20 s: Nav2 needs slam_toolbox's first map before its costmaps
activate, and the drive takes ~5 s at 0.15 m/s.
"""

import math
import os
import time
import unittest

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

NAV2_SERVERS = ['controller_server', 'planner_server', 'behavior_server',
                'bt_navigator', 'waypoint_follower']
# controllers, slam_toolbox's first map and the lifecycle bringup of all
# Nav2 servers, on a loaded CI machine
STARTUP_TIMEOUT = 90.0
# 0.5 m at 0.15 m/s plus turning and replanning slack
GOAL_TIMEOUT = 60.0
GOAL_X = 0.5
# xy_goal_tolerance in nav2_params.yaml plus slack for the final stop
GOAL_TOLERANCE = 0.15
# XV-11: 360 beams, ~5 Hz, 0.06..5 m
SCAN_PERIOD = 0.2
RANGE_MIN = 0.06
RANGE_MAX = 5.0
# room in odom: x from -1.5 to 2.5 m, y from -1.0 to 2.0 m
ROOM = (-1.5, 2.5, -1.0, 2.0)


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
            'slam': 'mapping',
            'nav': 'true',
        }.items())
    return LaunchDescription([
        bringup,
        launch_testing.actions.ReadyToTest(),
    ])


def room_range(x, y, angle):
    """Distance from (x, y) inside the room to its walls along angle."""
    x_min, x_max, y_min, y_max = ROOM
    dx, dy = math.cos(angle), math.sin(angle)
    hits = []
    if abs(dx) > 1e-9:
        hits.append(((x_max if dx > 0 else x_min) - x) / dx)
    if abs(dy) > 1e-9:
        hits.append(((y_max if dy > 0 else y_min) - y) / dy)
    return min(hits)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TestNav2(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_nav2')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.scans_published = 0
        self.node.create_timer(SCAN_PERIOD, self.publish_scan)
        self.odom = []
        self.node.create_subscription(Odometry, '/odom', self.odom.append,
                                      10)
        self.maps = []
        self.node.create_subscription(
            OccupancyGrid, '/map', self.maps.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.cmd_vel_nav = []
        self.node.create_subscription(TwistStamped, '/cmd_vel_nav',
                                      self.cmd_vel_nav.append, 10)

    def tearDown(self):
        self.tf_listener.unregister()
        self.node.destroy_node()

    def publish_scan(self):
        try:
            lidar = self.tf_buffer.lookup_transform('odom', 'lidar',
                                                    rclpy.time.Time())
        except TransformException:
            return
        x = lidar.transform.translation.x
        y = lidar.transform.translation.y
        yaw = yaw_of(lidar.transform.rotation)
        msg = LaserScan()
        # stamped like the pose it was computed from
        msg.header.stamp = lidar.header.stamp
        msg.header.frame_id = 'lidar'
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / 360
        msg.angle_max = msg.angle_increment * 359
        msg.scan_time = SCAN_PERIOD
        msg.time_increment = SCAN_PERIOD / 360
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = [
            room_range(x, y, yaw + msg.angle_min + i * msg.angle_increment)
            for i in range(360)]
        self.scan_pub.publish(msg)
        self.scans_published += 1

    def spin_until(self, condition, timeout):
        """Spin until condition() is true, False after timeout seconds."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def wait_future(self, future, timeout):
        self.assertTrue(self.spin_until(future.done, timeout),
                        'no response in time')
        return future.result()

    def inactive_servers(self, clients):
        inactive = []
        for name, client in clients.items():
            if not client.service_is_ready():
                inactive.append(name)
                continue
            # a response can get lost while discovery settles, the next
            # round asks again
            future = client.call_async(GetState.Request())
            if (not self.spin_until(future.done, 1.0)
                    or future.result().current_state.id
                    != State.PRIMARY_STATE_ACTIVE):
                inactive.append(name)
        return inactive

    def wait_nav2_active(self):
        clients = {name: self.node.create_client(GetState,
                                                 f'/{name}/get_state')
                   for name in NAV2_SERVERS}
        inactive = NAV2_SERVERS

        def all_active():
            nonlocal inactive
            inactive = self.inactive_servers(clients)
            return not inactive

        active = self.spin_until(all_active, STARTUP_TIMEOUT)
        for client in clients.values():
            self.node.destroy_client(client)
        self.assertTrue(active, f'Nav2 servers not active: {inactive} '
                        f'({self.scans_published} scans published)')

    def test_nav2_servers_become_active(self):
        self.wait_nav2_active()

    def test_goal_ahead_is_reached(self):
        self.wait_nav2_active()
        # a goal before slam_toolbox's first map only exercises recoveries
        self.assertTrue(self.spin_until(lambda: self.maps, STARTUP_TIMEOUT),
                        'no /map')
        self.assertTrue(self.spin_until(lambda: self.odom, 5.0), 'no /odom')
        start_x = self.odom[-1].pose.pose.position.x
        client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.assertTrue(client.wait_for_server(timeout_sec=10.0))

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = start_x + GOAL_X
        goal.pose.pose.orientation.w = 1.0
        handle = self.wait_future(client.send_goal_async(goal), 10.0)
        self.assertTrue(handle.accepted, 'goal rejected')
        result = self.wait_future(handle.get_result_async(), GOAL_TIMEOUT)

        self.assertTrue(
            any(abs(m.twist.linear.x) > 1e-3 for m in self.cmd_vel_nav),
            'no non-zero command on /cmd_vel_nav')
        self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED,
                         f'error {result.result.error_code}: '
                         f'{result.result.error_msg}')
        # mock odometry is exact, map == odom up to slam's corrections
        self.assertAlmostEqual(self.odom[-1].pose.pose.position.x,
                               start_x + GOAL_X, delta=GOAL_TOLERANCE)
        client.destroy()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_processes_exit_cleanly(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
