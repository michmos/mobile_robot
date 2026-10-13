import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

from mobile_robot.urdf_kinematics import write_kinematics_params


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            description='Use mock_components/GenericSystem instead of the '
                        'esp32 (no hardware needed)'),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Start the xv_11 lidar driver on /dev/ttyAMA0'),
        DeclareLaunchArgument(
            'use_foxglove',
            default_value='true',
            description='Start foxglove_bridge'),
        # the robot description is expanded in Python (the controller
        # kinematics are derived from it), so it needs the resolved value
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    use_mock_hardware = LaunchConfiguration(
        'use_mock_hardware').perform(context)

    xacro_file = os.path.join(
        get_package_share_directory('mobile_robot'),
        'description', 'mobile_robot.urdf.xacro')
    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={'use_mock_hardware': use_mock_hardware}).toxml()
    robot_description = {'robot_description': urdf_xml}
    kinematics_config = write_kinematics_params(urdf_xml)

    controller_manager_config = os.path.join(
        get_package_share_directory('mobile_robot'),
        'config', 'controller_manager.yaml')

    twist_mux_config = os.path.join(
        get_package_share_directory('mobile_robot'),
        'config', 'twist_mux.yaml')

    return [
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
            on_exit=Shutdown(reason='ros2_control_node exited'),
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
            condition=IfCondition(LaunchConfiguration('use_lidar')),
        ),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_foxglove')),
        ),
    ]
