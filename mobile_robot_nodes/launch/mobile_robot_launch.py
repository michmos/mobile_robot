
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch_ros.substitutions import
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
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200']
        )
    ])
