"""Smoke tests for the launch arguments of mobile_robot_launch.py."""

import importlib.util
import os
import tempfile

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import pytest

PACKAGE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir)
LAUNCH_FILE = os.path.join(PACKAGE_DIR, 'launch', 'mobile_robot_launch.py')


@pytest.fixture
def launch_module(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('mobile_robot_launch',
                                                  LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # read description/ and config/ from the source tree, so the test does
    # not depend on the install space
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: PACKAGE_DIR)
    # the launch file writes the kinematics params to the system temp dir
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    return module


def declared_defaults(launch_module):
    return {a.name: a.default_value[0].perform(LaunchContext())
            for a in launch_module.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)}


def launch_nodes(launch_module, **configurations):
    """Nodes the launch file starts for the given launch arguments."""
    context = LaunchContext()
    context.launch_configurations.update(declared_defaults(launch_module))
    context.launch_configurations.update(configurations)
    setup = next(a for a in launch_module.generate_launch_description()
                 .entities if isinstance(a, OpaqueFunction))
    nodes = setup.execute(context)
    return {n.node_executable: n for n in nodes
            if n.condition is None or n.condition.evaluate(context)}


def test_defaults_keep_robot_behavior(launch_module):
    assert declared_defaults(launch_module) == {
        'use_mock_hardware': 'false',
        'use_lidar': 'true',
        'use_foxglove': 'true',
    }


def test_defaults_start_lidar_and_foxglove(launch_module):
    nodes = launch_nodes(launch_module)

    assert 'xv_11_driver' in nodes
    assert 'foxglove_bridge' in nodes


def test_use_lidar_false_skips_lidar(launch_module):
    nodes = launch_nodes(launch_module, use_lidar='false')

    assert 'xv_11_driver' not in nodes
    assert 'foxglove_bridge' in nodes


def test_use_foxglove_false_skips_foxglove(launch_module):
    nodes = launch_nodes(launch_module, use_foxglove='false')

    assert 'foxglove_bridge' not in nodes
    assert 'xv_11_driver' in nodes


def test_dev_machine_setup_starts_only_ros2_control_stack(launch_module):
    nodes = launch_nodes(launch_module, use_mock_hardware='true',
                         use_lidar='false', use_foxglove='false')

    assert sorted(nodes) == ['robot_state_publisher', 'ros2_control_node',
                             'spawner', 'twist_mux']


@pytest.mark.parametrize('use_mock_hardware, plugin', [
    ('false', 'mobile_robot_hardware/MobileRobotHardware'),
    ('true', 'mock_components/GenericSystem'),
])
def test_use_mock_hardware_selects_hardware_plugin(launch_module,
                                                   monkeypatch,
                                                   use_mock_hardware,
                                                   plugin):
    descriptions = []
    process_file = launch_module.xacro.process_file

    def recording_process_file(*args, **kwargs):
        document = process_file(*args, **kwargs)
        descriptions.append(document.toxml())
        return document
    monkeypatch.setattr(launch_module.xacro, 'process_file',
                        recording_process_file)

    launch_nodes(launch_module, use_mock_hardware=use_mock_hardware)

    assert len(descriptions) == 1
    assert f'<plugin>{plugin}</plugin>' in descriptions[0]
