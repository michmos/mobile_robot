import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('mobile_robot'),
        'description', 'mobile_robot.urdf.xacro')
    robot_description = {
        'robot_description': xacro.process_file(xacro_file).toxml()
    }

    controller_manager_config = os.path.join(
        get_package_share_directory('mobile_robot'),
        'config', 'controller_manager.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='both',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, controller_manager_config],
            output='both',
        ),
        # Spawn controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mobile_robot_controller'],
            output='screen',
        ),
    ])
