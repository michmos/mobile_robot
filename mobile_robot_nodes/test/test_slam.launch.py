"""
Integration test: slam_launch.py mode:=mapping on its own.

Starts slam_launch.py with static odom -> base_link -> lidar transforms (what
mobile_robot_controller and robot_state_publisher publish on the robot) and
a fake XV-11 /scan of a rectangular room, and checks that slam_toolbox
publishes map -> odom and /map, and that save_map writes the pose graph and
the occupancy grid.
"""

import math
import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from mobile_robot.save_map import save
from nav_msgs.msg import OccupancyGrid
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

# slam_toolbox configures, activates and processes the first scan; /map is
# published every map_update_interval (5 s)
STARTUP_TIMEOUT = 30.0
# XV-11: 360 beams, ~5 Hz, 0.06..5 m
SCAN_PERIOD = 0.2
RANGE_MIN = 0.06
RANGE_MAX = 5.0
# room around the lidar: x from -1.5 to 2.5 m, y from -1.0 to 2.0 m
ROOM = (-1.5, 2.5, -1.0, 2.0)


@pytest.mark.launch_test
def generate_test_description():
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('mobile_robot'),
            'launch', 'slam_launch.py')),
        launch_arguments={'mode': 'mapping'}.items())
    return LaunchDescription([
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'odom', '--child-frame-id',
                        'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--z', '0.1', '--frame-id', 'base_link',
                        '--child-frame-id', 'lidar']),
        slam,
        launch_testing.actions.ReadyToTest(),
    ])


def room_range(angle):
    """Distance from the origin to the room's walls along angle."""
    x_min, x_max, y_min, y_max = ROOM
    dx, dy = math.cos(angle), math.sin(angle)
    hits = []
    if abs(dx) > 1e-9:
        hits.append((x_max if dx > 0 else x_min) / dx)
    if abs(dy) > 1e-9:
        hits.append((y_max if dy > 0 else y_min) / dy)
    return min(hits)


class TestSlamMapping(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_slam')
        self.maps = []
        self.node.create_subscription(
            OccupancyGrid, '/map', self.maps.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.node.create_timer(SCAN_PERIOD, self.publish_scan)

    def tearDown(self):
        self.tf_listener.unregister()
        self.node.destroy_node()

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar'
        msg.angle_min = 0.0
        msg.angle_increment = 2.0 * math.pi / 360
        msg.angle_max = msg.angle_increment * 359
        msg.scan_time = SCAN_PERIOD
        msg.time_increment = SCAN_PERIOD / 360
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = [room_range(msg.angle_min + i * msg.angle_increment)
                      for i in range(360)]
        self.scan_pub.publish(msg)

    def spin_until(self, condition, timeout=STARTUP_TIMEOUT):
        """Spin until condition() is true, False after timeout seconds."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return condition()

    def map_to_odom(self):
        try:
            return self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time())
        except TransformException:
            return None

    def test_map_to_odom_is_published(self):
        self.assertTrue(self.spin_until(self.map_to_odom),
                        'no map -> odom transform')

    def test_map_is_published(self):
        self.assertTrue(self.spin_until(lambda: self.maps), 'no /map')
        grid = self.maps[-1]

        self.assertEqual(grid.header.frame_id, 'map')
        self.assertAlmostEqual(grid.info.resolution, 0.05, places=6)
        # the walls of the fake room
        self.assertIn(100, grid.data)

    def test_save_map_writes_pose_graph_and_grid(self):
        self.assertTrue(self.spin_until(lambda: self.maps), 'no /map')
        with tempfile.TemporaryDirectory() as map_dir:
            base_path = os.path.join(map_dir, 'room')

            self.assertTrue(save(self.node, base_path))

            for suffix in ('.posegraph', '.data', '.pgm', '.yaml'):
                self.assertTrue(os.path.isfile(base_path + suffix),
                                f'no {base_path}{suffix}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # static_transform_publisher and slam_toolbox exit cleanly on SIGINT
        launch_testing.asserts.assertExitCodes(proc_info)
