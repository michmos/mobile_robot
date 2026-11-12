import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import LifecycleNode
from mobile_robot.map_storage import (DEFAULT_MAP_DIR, map_base_path,
                                      MAP_DIR_ENV, POSE_GRAPH_SUFFIX)

# mode -> (slam_toolbox executable, config file)
MODES = {
    'mapping': ('async_slam_toolbox_node', 'slam_mapping.yaml'),
    'localization': ('localization_slam_toolbox_node',
                     'slam_localization.yaml'),
}


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='mapping',
            choices=list(MODES),
            description='mapping: build a map (online async), '
                        'localization: localize in the saved map `map`'),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Map name: loaded in localization mode (required), '
                        'in mapping mode only used for the save hint'),
        DeclareLaunchArgument(
            'map_dir',
            default_value=EnvironmentVariable(
                MAP_DIR_ENV, default_value=DEFAULT_MAP_DIR),
            description=f'Directory holding the maps, '
                        f'${MAP_DIR_ENV} or {DEFAULT_MAP_DIR} by default'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        # the map path is checked in Python, so it needs the resolved values
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    mode = LaunchConfiguration('mode').perform(context)
    map_name = LaunchConfiguration('map').perform(context)
    map_dir = LaunchConfiguration('map_dir').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')

    executable, config_name = MODES[mode]
    config = os.path.join(get_package_share_directory('mobile_robot'),
                          'config', config_name)
    parameters = [config, {'use_sim_time': use_sim_time}]
    actions = []

    if mode == 'localization':
        if not map_name:
            raise RuntimeError('slam mode localization needs map:=<name>')
        base_path = map_base_path(map_name, map_dir)
        # fail here instead of only in slam_toolbox's log
        if not os.path.isfile(base_path + POSE_GRAPH_SUFFIX):
            raise RuntimeError(f"map '{map_name}' not found: no "
                               f'{base_path}{POSE_GRAPH_SUFFIX}')
        parameters.append({'map_file_name': base_path})
    elif map_name:
        # validates the name before the user has driven the whole map
        map_base_path(map_name, map_dir)
        actions.append(LogInfo(
            msg=f'save the map with: ros2 run mobile_robot save_map '
                f'{map_name} --map-dir {map_dir}'))

    actions.append(LifecycleNode(
        package='slam_toolbox',
        executable=executable,
        name='slam_toolbox',
        namespace='',
        parameters=parameters,
        autostart=True,
        output='screen',
    ))
    return actions
