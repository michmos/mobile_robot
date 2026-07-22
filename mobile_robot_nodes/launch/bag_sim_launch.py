
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
#


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot',
            executable='control_node',
            name='motor_controler'
        ),
        Node(
            package='mobile_robot',
            executable='perception_node',
            name='perception'
        ),
        Node(
            package='mobile_robot',
            executable='collision_avoid_node',
            name='collision_avoid'
        ),
        ExecuteProcess(cmd=['ros2', 'bag', 'play', '-l', PathJoinSubstitution(
            [FindPackageShare('mobile_robot'), 'data', 'sensor_recording'])])
    ])
