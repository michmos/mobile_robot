"""
Smoke tests for slam_launch.py and the slam argument of mobile_robot_launch.py.

Only the launch descriptions are built, no node is started.
"""

import importlib.util
import os
import tempfile

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.utilities import perform_substitutions
from launch_ros.actions import LifecycleNode, Node
from launch_ros.utilities import evaluate_parameters
import pytest

PACKAGE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir)
LAUNCH_DIR = os.path.join(PACKAGE_DIR, 'launch')
CONFIG_DIR = os.path.join(PACKAGE_DIR, 'config')


def load_module(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(
        name[:-3], os.path.join(LAUNCH_DIR, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # read config/ from the source tree, not the install space
    monkeypatch.setattr(module, 'get_package_share_directory',
                        lambda package: PACKAGE_DIR)
    return module


@pytest.fixture
def slam_launch(monkeypatch):
    monkeypatch.delenv('MAP_DIR', raising=False)
    return load_module('slam_launch.py', monkeypatch)


@pytest.fixture
def robot_launch(tmp_path, monkeypatch):
    monkeypatch.delenv('MAP_DIR', raising=False)
    # the launch file writes the kinematics params to the system temp dir
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    return load_module('mobile_robot_launch.py', monkeypatch)


def declared_arguments(module):
    return [a for a in module.generate_launch_description().entities
            if isinstance(a, DeclareLaunchArgument)]


def declared_defaults(module):
    return {a.name: perform_substitutions(LaunchContext(), a.default_value)
            for a in declared_arguments(module)}


def run(module, **configurations):
    """Actions the launch file starts for the given launch arguments."""
    context = LaunchContext()
    context.launch_configurations.update(declared_defaults(module))
    context.launch_configurations.update(configurations)
    # validates the values against the declared choices
    for argument in declared_arguments(module):
        argument.execute(context)
    setup = next(a for a in module.generate_launch_description().entities
                 if isinstance(a, OpaqueFunction))
    return context, [a for a in setup.execute(context)
                     if a.condition is None or a.condition.evaluate(context)]


def slam_node(actions):
    nodes = [a for a in actions if isinstance(a, LifecycleNode)]
    assert len(nodes) == 1
    return nodes[0]


# launch_ros has no public getters for these before the node is executed

def node_parameters(context, node):
    """Parameter files and the merged dict parameters of a node."""
    files, values = [], {}
    for parameter in evaluate_parameters(context, node._Node__parameters):
        if isinstance(parameter, dict):
            values.update(parameter)
        else:
            files.append(str(parameter))
    return files, values


def node_name(node):
    return node._Node__node_name


def make_map(map_dir, name):
    folder = map_dir / name
    folder.mkdir(parents=True)
    (folder / f'{name}.posegraph').write_text('')
    (folder / f'{name}.data').write_text('')


# slam_launch.py

def test_slam_launch_defaults(slam_launch):
    assert declared_defaults(slam_launch) == {
        'mode': 'mapping',
        'map': '',
        'map_dir': '~/maps',
        'use_sim_time': 'false',
    }


def test_slam_launch_map_dir_defaults_to_env(slam_launch, monkeypatch):
    monkeypatch.setenv('MAP_DIR', '/data/maps')

    assert declared_defaults(slam_launch)['map_dir'] == '/data/maps'


def test_mapping_starts_async_node_with_mapping_config(slam_launch):
    context, actions = run(slam_launch, mode='mapping')
    node = slam_node(actions)
    files, values = node_parameters(context, node)

    assert node.node_package == 'slam_toolbox'
    assert node.node_executable == 'async_slam_toolbox_node'
    # the services save_map calls are /slam_toolbox/...
    assert node_name(node) == 'slam_toolbox'
    assert files == [os.path.join(CONFIG_DIR, 'slam_mapping.yaml')]
    assert 'map_file_name' not in values


def test_mapping_with_map_name_still_starts_a_new_map(slam_launch, tmp_path):
    make_map(tmp_path, 'kitchen')

    context, actions = run(slam_launch, mode='mapping', map='kitchen',
                           map_dir=str(tmp_path))

    assert 'map_file_name' not in \
        node_parameters(context, slam_node(actions))[1]


def test_localization_loads_pose_graph_of_map(slam_launch, tmp_path):
    make_map(tmp_path, 'kitchen')

    context, actions = run(slam_launch, mode='localization', map='kitchen',
                           map_dir=str(tmp_path))
    node = slam_node(actions)
    files, values = node_parameters(context, node)

    assert node.node_executable == 'localization_slam_toolbox_node'
    assert files == [os.path.join(CONFIG_DIR, 'slam_localization.yaml')]
    # slam_toolbox appends .posegraph/.data itself
    assert values['map_file_name'] == str(tmp_path / 'kitchen' / 'kitchen')


@pytest.mark.parametrize('mode', ['mapping', 'localization'])
def test_slam_node_is_activated_by_the_launch(slam_launch, tmp_path, mode):
    # slam_toolbox is a lifecycle node and does nothing until active
    make_map(tmp_path, 'kitchen')

    _, actions = run(slam_launch, mode=mode, map='kitchen',
                     map_dir=str(tmp_path))

    assert slam_node(actions).node_autostart is True


def test_use_sim_time_is_passed_to_slam_toolbox(slam_launch):
    context, actions = run(slam_launch, use_sim_time='true')

    assert node_parameters(context, slam_node(actions))[1][
        'use_sim_time'] is True


def test_localization_without_map_fails(slam_launch):
    with pytest.raises(RuntimeError, match='map:='):
        run(slam_launch, mode='localization')


def test_localization_with_missing_map_fails(slam_launch, tmp_path):
    with pytest.raises(RuntimeError, match='not found'):
        run(slam_launch, mode='localization', map='kitchen',
            map_dir=str(tmp_path))


def test_invalid_map_name_fails(slam_launch, tmp_path):
    with pytest.raises(ValueError):
        run(slam_launch, mode='mapping', map='../kitchen',
            map_dir=str(tmp_path))


def test_unknown_mode_is_rejected(slam_launch):
    with pytest.raises(Exception, match='not valid'):
        run(slam_launch, mode='lifelong')


# mobile_robot_launch.py slam argument

def slam_includes(actions):
    return [a for a in actions if isinstance(a, IncludeLaunchDescription)]


def test_robot_launch_slam_defaults(robot_launch):
    defaults = declared_defaults(robot_launch)

    assert {k: defaults[k] for k in ('slam', 'map', 'map_dir',
                                     'use_sim_time')} == {
        'slam': 'off', 'map': '', 'map_dir': '~/maps',
        'use_sim_time': 'false'}


def test_robot_launch_slam_off_starts_no_slam_toolbox(robot_launch):
    _, actions = run(robot_launch, slam='off')

    assert slam_includes(actions) == []
    assert not [a for a in actions if isinstance(a, Node)
                and a.node_package == 'slam_toolbox']


@pytest.mark.parametrize('slam', ['mapping', 'localization'])
def test_robot_launch_slam_includes_slam_launch(robot_launch, slam):
    context, actions = run(robot_launch, slam=slam, map='kitchen',
                           map_dir='/data/maps', use_sim_time='true')
    includes = slam_includes(actions)

    assert len(includes) == 1
    source = includes[0].launch_description_source
    # resolves the location
    source.get_launch_description(context)
    assert source.location == \
        os.path.join(PACKAGE_DIR, 'launch', 'slam_launch.py')
    assert {k: perform_substitutions(context, [v])
            for k, v in includes[0].launch_arguments} == {
        'mode': slam, 'map': 'kitchen', 'map_dir': '/data/maps',
        'use_sim_time': 'true'}


def test_robot_launch_unknown_slam_mode_is_rejected(robot_launch):
    with pytest.raises(Exception, match='not valid'):
        run(robot_launch, slam='on')
