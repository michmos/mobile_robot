"""
Smoke tests for nav2_launch.py and the nav argument of mobile_robot_launch.py.

Only the launch descriptions are built, no node is started.
"""

import importlib.util
import os
import tempfile

from launch import LaunchContext
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
import pytest

PACKAGE_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir)
LAUNCH_DIR = os.path.join(PACKAGE_DIR, 'launch')
CONFIG_DIR = os.path.join(PACKAGE_DIR, 'config')

NAV2_SERVERS = ['controller_server', 'planner_server', 'behavior_server',
                'bt_navigator', 'waypoint_follower']


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
def nav2_launch(monkeypatch):
    return load_module('nav2_launch.py', monkeypatch)


@pytest.fixture
def robot_launch(tmp_path, monkeypatch):
    monkeypatch.delenv('MAP_DIR', raising=False)
    # the launch file writes the kinematics params to the system temp dir
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    return load_module('mobile_robot_launch.py', monkeypatch)


def declared_arguments(description):
    return [a for a in description.entities
            if isinstance(a, DeclareLaunchArgument)]


def context_with_defaults(description, **configurations):
    context = LaunchContext()
    for argument in declared_arguments(description):
        context.launch_configurations[argument.name] = perform_substitutions(
            context, argument.default_value)
    context.launch_configurations.update(configurations)
    # validates the values against the declared choices
    for argument in declared_arguments(description):
        argument.execute(context)
    return context


# launch_ros has no public getters for these before the node is executed

def node_name(node):
    return node._Node__node_name


def node_parameters(context, node):
    """Parameter files and the merged dict parameters of a node."""
    files, values = [], {}
    for parameter in evaluate_parameters(context, node._Node__parameters):
        if isinstance(parameter, dict):
            values.update(parameter)
        else:
            files.append(str(parameter))
    return files, values


def node_remappings(context, node):
    return {perform_substitutions(context, src):
            perform_substitutions(context, dst)
            for src, dst in node._Node__remappings}


# nav2_launch.py

@pytest.fixture
def nav2_nodes(nav2_launch):
    description = nav2_launch.generate_launch_description()
    context = context_with_defaults(description)
    return context, {node_name(n): n for n in description.entities
                     if isinstance(n, Node)}


def test_nav2_launch_defaults(nav2_launch):
    description = nav2_launch.generate_launch_description()

    assert {a.name: perform_substitutions(LaunchContext(), a.default_value)
            for a in declared_arguments(description)} == {
        'use_sim_time': 'false',
        'params_file': os.path.join(CONFIG_DIR, 'nav2_params.yaml'),
    }


def test_nav2_launch_starts_servers_and_one_lifecycle_manager(nav2_nodes):
    _, nodes = nav2_nodes

    assert sorted(nodes) == sorted(
        NAV2_SERVERS + ['lifecycle_manager_navigation'])


def test_lifecycle_manager_activates_every_server(nav2_nodes):
    context, nodes = nav2_nodes
    _, values = node_parameters(context,
                                nodes['lifecycle_manager_navigation'])

    assert values['autostart'] is True
    assert sorted(values['node_names']) == sorted(NAV2_SERVERS)


@pytest.mark.parametrize('server', NAV2_SERVERS)
def test_servers_load_the_params_file(nav2_nodes, server):
    context, nodes = nav2_nodes

    files, values = node_parameters(context, nodes[server])

    assert files == [os.path.join(CONFIG_DIR, 'nav2_params.yaml')]
    assert values['use_sim_time'] is False


@pytest.mark.parametrize('server', ['controller_server', 'behavior_server'])
def test_velocity_output_goes_to_twist_mux_nav_input(nav2_nodes, server):
    context, nodes = nav2_nodes

    assert node_remappings(context, nodes[server])['cmd_vel'] == \
        '/cmd_vel_nav'


def test_use_sim_time_is_passed_to_nav2(nav2_launch):
    description = nav2_launch.generate_launch_description()
    context = context_with_defaults(description, use_sim_time='true')

    for node in description.entities:
        if isinstance(node, Node):
            assert node_parameters(context, node)[1]['use_sim_time'] is True


# mobile_robot_launch.py nav argument

def run(module, **configurations):
    """Actions the launch file starts for the given launch arguments."""
    description = module.generate_launch_description()
    context = context_with_defaults(description, **configurations)
    setup = next(a for a in description.entities
                 if isinstance(a, OpaqueFunction))
    return context, [a for a in setup.execute(context)
                     if a.condition is None or a.condition.evaluate(context)]


def nav2_includes(context, actions):
    includes = []
    for action in actions:
        if not isinstance(action, IncludeLaunchDescription):
            continue
        source = action.launch_description_source
        # resolves the location
        source.get_launch_description(context)
        if os.path.basename(source.location) == 'nav2_launch.py':
            includes.append(action)
    return includes


def test_robot_launch_nav_defaults_to_false(robot_launch):
    description = robot_launch.generate_launch_description()

    assert context_with_defaults(description).launch_configurations[
        'nav'] == 'false'


@pytest.mark.parametrize('slam', ['off', 'mapping', 'localization'])
def test_robot_launch_without_nav_starts_no_nav2(robot_launch, slam):
    context, actions = run(robot_launch, slam=slam, map='kitchen')

    assert nav2_includes(context, actions) == []


@pytest.mark.parametrize('slam', ['mapping', 'localization'])
def test_robot_launch_nav_includes_nav2_launch(robot_launch, slam):
    context, actions = run(robot_launch, slam=slam, map='kitchen',
                           nav='true', use_sim_time='true')
    includes = nav2_includes(context, actions)

    assert len(includes) == 1
    assert includes[0].launch_description_source.location == \
        os.path.join(LAUNCH_DIR, 'nav2_launch.py')
    assert {k: perform_substitutions(context, [v])
            for k, v in includes[0].launch_arguments} == {
        'use_sim_time': 'true'}


def test_robot_launch_nav_without_slam_fails(robot_launch):
    # Nav2 needs map -> odom and /map from slam_toolbox
    with pytest.raises(RuntimeError, match='slam:='):
        run(robot_launch, slam='off', nav='true')


def test_robot_launch_rejects_invalid_nav_value(robot_launch):
    with pytest.raises(Exception, match='not valid'):
        run(robot_launch, slam='mapping', nav='yes')
