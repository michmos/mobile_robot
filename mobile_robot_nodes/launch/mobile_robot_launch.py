
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
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200']
        )
    ])
