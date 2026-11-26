import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Nav2 servers, brought up in this order by the lifecycle manager
NAV2_NODES = [
    ('nav2_controller', 'controller_server'),
    ('nav2_planner', 'planner_server'),
    ('nav2_behaviors', 'behavior_server'),
    ('nav2_bt_navigator', 'bt_navigator'),
    ('nav2_waypoint_follower', 'waypoint_follower'),
]
# twist_mux input with the lowest priority, see config/twist_mux.yaml
CMD_VEL_TOPIC = '/cmd_vel_nav'


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    params = [params_file, {'use_sim_time': use_sim_time}]

    nav2_nodes = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            parameters=params,
            # controller_server and behavior_server publish on cmd_vel
            remappings=[('cmd_vel', CMD_VEL_TOPIC)],
            output='screen',
        )
        for package, executable in NAV2_NODES
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                get_package_share_directory('mobile_robot'),
                'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        *nav2_nodes,
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [name for _, name in NAV2_NODES],
            }],
            output='screen',
        ),
    ])
