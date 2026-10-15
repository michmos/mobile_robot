"""
Smoke tests for the robot description, configs, launch files and plugin.xml.

Cheap static checks that catch typos on the dev machine instead of on the
robot. Nothing is started: launch files are only imported and their
LaunchDescription inspected.
"""

import importlib.util
import os
import re
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest
from urdf_parser_py.urdf import URDF
import xacro
import yaml

PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
REPO_DIR = os.path.join(PKG_DIR, os.pardir)

XACRO_FILE = os.path.join(PKG_DIR, 'description', 'mobile_robot.urdf.xacro')
CONFIG_DIR = os.path.join(PKG_DIR, 'config')
LAUNCH_DIR = os.path.join(PKG_DIR, 'launch')
PARAMS_CPP = os.path.join(REPO_DIR, 'mobile_robot_hardware', 'src',
                          'params.cpp')

# The required <param> names parseParams() reads in
# mobile_robot_hardware/src/params.cpp. Keep in sync with it;
# test_hardware_param_list_matches_parse_params guards against drift.
HARDWARE_PARAMS = {
    'serial_port',
    'baud_rate',
    'pid_kp',
    'pid_ki',
    'pid_integral_limit',
    'ff_slope_left',
    'ff_slope_right',
    'ff_min_duty_left',
    'ff_min_duty_right',
    'ticks_per_wheel_rev',
    'watchdog_timeout_ms',
    'control_rate_hz',
    'report_rate_hz',
    'invert_left',
    'invert_right',
}

# Params parseParams() reads only if present (with a default otherwise).
OPTIONAL_HARDWARE_PARAMS = {
    'handshake_timeout_ms',
}

WHEEL_JOINTS = ['left_wheel_joint', 'right_wheel_joint']

# Hardware plugin per value of the use_mock_hardware xacro arg.
HARDWARE_PLUGINS = {
    'false': 'mobile_robot_hardware/MobileRobotHardware',
    'true': 'mock_components/GenericSystem',
}

# Controller types from ros2_controllers this repo may use. Types not in
# here must be exported by a plugin.xml in this repo.
KNOWN_ROS2_CONTROLLERS_TYPES = {
    'joint_state_broadcaster/JointStateBroadcaster',
    'diff_drive_controller/DiffDriveController',
    'forward_command_controller/ForwardCommandController',
    'velocity_controllers/JointGroupVelocityController',
    'imu_sensor_broadcaster/IMUSensorBroadcaster',
}

PLUGIN_PACKAGES = ['mobile_robot_hardware', 'mobile_robot_controller']


# helpers

def expand_xacro(use_mock_hardware):
    return xacro.process_file(
        XACRO_FILE,
        mappings={'use_mock_hardware': use_mock_hardware}).toxml()


def ros2_control_block(urdf_xml):
    blocks = ET.fromstring(urdf_xml).findall('ros2_control')
    assert len(blocks) == 1
    return blocks[0]


@pytest.fixture(scope='module', params=sorted(HARDWARE_PLUGINS))
def use_mock_hardware(request):
    return request.param


@pytest.fixture(scope='module')
def urdf_xml(use_mock_hardware):
    # The description must hold on the robot and with the mock hardware.
    return expand_xacro(use_mock_hardware)


@pytest.fixture(scope='module')
def ros2_control(urdf_xml):
    return ros2_control_block(urdf_xml)


@pytest.fixture(scope='module')
def esp32_ros2_control():
    """<ros2_control> of the real robot; the mock has no esp32 params."""
    return ros2_control_block(expand_xacro('false'))


def load_yaml(name):
    with open(os.path.join(CONFIG_DIR, name)) as file:
        return yaml.safe_load(file)


def hardware_params(ros2_control):
    return {p.get('name'): p.text for p in ros2_control.findall(
        'hardware/param')}


def plugin_xml_classes(package):
    root = ET.parse(os.path.join(REPO_DIR, package, 'plugin.xml')).getroot()
    return root, root.findall('class')


def exported_classes(package):
    """(class type, base class type) of each PLUGINLIB_EXPORT_CLASS."""
    exports = []
    src_dir = os.path.join(REPO_DIR, package, 'src')
    for name in sorted(os.listdir(src_dir)):
        with open(os.path.join(src_dir, name)) as file:
            exports += re.findall(
                r'PLUGINLIB_EXPORT_CLASS\(\s*([\w:]+)\s*,\s*([\w:]+)\s*\)',
                file.read())
    return exports


def load_launch_description(name, tmp_path, monkeypatch):
    # mobile_robot_launch.py writes the kinematics params to the system temp
    # dir; redirect it so no file is left behind.
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    spec = importlib.util.spec_from_file_location(
        name[:-3], os.path.join(LAUNCH_DIR, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # read description/ and config/ from the source tree
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: PKG_DIR)
    return module.generate_launch_description()


def launch_entities(description):
    """Top-level actions, with OpaqueFunctions run at the default args."""
    context = LaunchContext()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(
                context, action.default_value)
    entities = []
    for action in description.entities:
        if isinstance(action, OpaqueFunction):
            entities += action.execute(context) or []
        else:
            entities.append(action)
    return [e for e in entities
            if e.condition is None or e.condition.evaluate(context)]


def resolve(value):
    """Text of a launch_ros Node field (str or list of substitutions)."""
    if isinstance(value, str):
        return value
    return perform_substitutions(LaunchContext(), value)


def nodes(description):
    """(package, executable, spawner arguments) of each Node in a launch."""
    result = []
    for action in launch_entities(description):
        if not isinstance(action, Node):
            continue
        # cmd[0] is the executable lookup (needs the package installed);
        # the user arguments follow it, up to the ROS specific ones.
        args = []
        for arg in action.cmd[1:]:
            text = resolve(arg)
            if text.startswith('--'):
                break
            args.append(text)
        result.append((resolve(action.node_package),
                       resolve(action.node_executable), tuple(args)))
    return result


# xacro / URDF

def test_xacro_processes_and_parses_as_urdf(urdf_xml):
    robot = URDF.from_xml_string(urdf_xml)

    assert robot.name == 'mobile_robot'
    assert robot.get_root() == 'base_link'


def test_urdf_has_expected_links(urdf_xml):
    robot = URDF.from_xml_string(urdf_xml)

    assert {link.name for link in robot.links} == {
        'base_link', 'chassis', 'left_wheel', 'right_wheel', 'caster_wheel',
        'imu', 'lidar'}


def test_urdf_has_expected_joints(urdf_xml):
    robot = URDF.from_xml_string(urdf_xml)

    # (parent, child, type); the lidar link is the frame_id the xv_11_driver
    # publishes in, see mobile_robot_launch.py.
    assert {j.name: (j.parent, j.child, j.type) for j in robot.joints} == {
        'base_to_chassis': ('base_link', 'chassis', 'fixed'),
        'left_wheel_joint': ('base_link', 'left_wheel', 'continuous'),
        'right_wheel_joint': ('base_link', 'right_wheel', 'continuous'),
        'caster_wheel_joint': ('chassis', 'caster_wheel', 'fixed'),
        'imu_joint': ('chassis', 'imu', 'fixed'),
        'lidar_joint': ('chassis', 'lidar', 'fixed'),
    }


def test_ros2_control_uses_expected_hardware_plugin(use_mock_hardware,
                                                    ros2_control):
    assert ros2_control.get('type') == 'system'
    assert ros2_control.findtext('hardware/plugin') == \
        HARDWARE_PLUGINS[use_mock_hardware]


def test_ros2_control_has_wheel_joints_with_expected_interfaces(
        ros2_control):
    # mobile_robot_controller claims velocity commands and reads position
    # (odometry) and velocity states of both wheels.
    joints = {
        j.get('name'): (
            sorted(c.get('name') for c in j.findall('command_interface')),
            sorted(s.get('name') for s in j.findall('state_interface')))
        for j in ros2_control.findall('joint')}

    assert joints == {name: (['velocity'], ['position', 'velocity'])
                      for name in WHEEL_JOINTS}


def test_ros2_control_joints_exist_in_urdf(urdf_xml, ros2_control):
    robot = URDF.from_xml_string(urdf_xml)

    for joint in ros2_control.findall('joint'):
        assert joint.get('name') in robot.joint_map


# hardware params

def test_ros2_control_hardware_params_match_parse_params(
        esp32_ros2_control):
    # A missing param makes on_init fail on the robot; an extra one is
    # silently ignored (likely a typo).
    params = set(hardware_params(esp32_ros2_control))

    assert HARDWARE_PARAMS <= params
    assert params - HARDWARE_PARAMS <= OPTIONAL_HARDWARE_PARAMS


def test_hardware_param_list_matches_parse_params():
    with open(PARAMS_CPP) as file:
        source = file.read()
    parse_params = source[source.index('parseParams(const'):]
    parse_params = parse_params[:parse_params.index('\n}\n')]

    keys = set(re.findall(r'parseParam<[\w:]+>\(\s*p\s*,\s*"(\w+)"\)',
                          parse_params))

    assert keys == HARDWARE_PARAMS | OPTIONAL_HARDWARE_PARAMS


# YAML configs

@pytest.mark.parametrize('name', [
    'controller_manager.yaml', 'twist_mux.yaml', 'joystick.yaml'])
def test_yaml_config_loads(name):
    config = load_yaml(name)

    assert isinstance(config, dict)
    for node_config in config.values():
        assert 'ros__parameters' in node_config


def test_controller_types_are_exported_or_known():
    params = load_yaml('controller_manager.yaml')[
        'controller_manager']['ros__parameters']
    exported = {c.get('name') for package in PLUGIN_PACKAGES
                for c in plugin_xml_classes(package)[1]}
    types = {name: value['type'] for name, value in params.items()
             if isinstance(value, dict) and 'type' in value}

    assert types == {
        'joint_state_broadcaster':
            'joint_state_broadcaster/JointStateBroadcaster',
        'mobile_robot_controller':
            'mobile_robot_controller/MobileRobotController',
    }
    for controller_type in types.values():
        assert controller_type in exported | KNOWN_ROS2_CONTROLLERS_TYPES


def test_controller_manager_update_rate_matches_hardware_control_rate(
        esp32_ros2_control):
    # controller_manager.yaml claims both loops run at the same rate.
    params = load_yaml('controller_manager.yaml')[
        'controller_manager']['ros__parameters']

    assert params['update_rate'] == int(
        hardware_params(esp32_ros2_control)['control_rate_hz'])


def test_controller_joint_names_exist_in_ros2_control(ros2_control):
    params = load_yaml('controller_manager.yaml')[
        'mobile_robot_controller']['ros__parameters']
    joints = [j.get('name') for j in ros2_control.findall('joint')]

    assert [params['left_wheel_joint_name'],
            params['right_wheel_joint_name']] == joints


def test_twist_mux_and_teleop_agree_on_stamped_twist():
    # twist_mux drops messages of the wrong type without an error.
    twist_mux = load_yaml('twist_mux.yaml')['twist_mux']['ros__parameters']
    teleop = load_yaml('joystick.yaml')['teleop_node']['ros__parameters']

    assert twist_mux['use_stamped'] == teleop['publish_stamped_twist']


# plugin.xml

@pytest.mark.parametrize('package', PLUGIN_PACKAGES)
def test_plugin_xml_matches_pluginlib_export(package):
    library, classes = plugin_xml_classes(package)
    exports = exported_classes(package)

    assert library.get('path') == package
    assert [(c.get('type'), c.get('base_class_type')) for c in classes] == \
        exports
    for c in classes:
        # pluginlib name convention: <package>/<Class>
        assert c.get('name') == \
            f"{package}/{c.get('type').split('::')[-1]}"


# launch files

def test_mobile_robot_launch_has_expected_nodes(tmp_path, monkeypatch):
    description = load_launch_description('mobile_robot_launch.py',
                                          tmp_path, monkeypatch)

    assert isinstance(description, LaunchDescription)
    assert sorted(nodes(description)) == sorted([
        ('robot_state_publisher', 'robot_state_publisher', ()),
        ('controller_manager', 'ros2_control_node', ()),
        ('controller_manager', 'spawner', ('joint_state_broadcaster',)),
        ('controller_manager', 'spawner', ('mobile_robot_controller',)),
        ('twist_mux', 'twist_mux', ()),
        ('xv_11_driver', 'xv_11_driver', ()),
        ('foxglove_bridge', 'foxglove_bridge', ()),
    ])


def test_mobile_robot_launch_spawns_every_configured_controller(
        tmp_path, monkeypatch):
    description = load_launch_description('mobile_robot_launch.py',
                                          tmp_path, monkeypatch)
    params = load_yaml('controller_manager.yaml')[
        'controller_manager']['ros__parameters']
    configured = {name for name, value in params.items()
                  if isinstance(value, dict) and 'type' in value}

    spawned = {arg for package, executable, args in nodes(description)
               if executable == 'spawner' for arg in args}

    assert spawned == configured


def test_joystick_launch_has_expected_nodes(tmp_path, monkeypatch):
    description = load_launch_description('joystick_launch.py',
                                          tmp_path, monkeypatch)

    assert isinstance(description, LaunchDescription)
    assert sorted(nodes(description)) == sorted([
        ('joy', 'joy_node', ()),
        ('teleop_twist_joy', 'teleop_node', ()),
    ])
    assert [a.name for a in description.entities
            if isinstance(a, DeclareLaunchArgument)] == ['use_sim_time']
