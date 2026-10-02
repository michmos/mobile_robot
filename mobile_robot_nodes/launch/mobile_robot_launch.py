import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

from mobile_robot.urdf_kinematics import write_kinematics_params


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('mobile_robot'),
        'description', 'mobile_robot.urdf.xacro')
    urdf_xml = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': urdf_xml}
    kinematics_config = write_kinematics_params(urdf_xml)

    controller_manager_config = os.path.join(
        get_package_share_directory('mobile_robot'),
        'config', 'controller_manager.yaml')

    twist_mux_config = os.path.join(
        get_package_share_directory('mobile_robot'),
        'config', 'twist_mux.yaml')

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
            parameters=[robot_description, controller_manager_config,
                        kinematics_config],
            output='both',
        ),
        # Spawn controllers
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mobile_robot_controller'],
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[twist_mux_config],
            remappings={('/cmd_vel_out', '/cmd_vel')},
            output='screen',
        ),
        Node(
            package='xv_11_driver',
            executable='xv_11_driver',
            parameters=[{'frame_id': 'lidar', 'port': '/dev/ttyAMA0'}],
            output='screen',
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            output='screen',
        ),
    ])
