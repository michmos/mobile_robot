"""
Smoke tests for nav2_launch.py and the nav argument of mobile_robot_launch.py.

Only the launch descriptions are built, no node is started.
"""

import os

from launch.actions import IncludeLaunchDescription
from launch.utilities import perform_substitutions
from launch_helpers import (CONFIG_DIR, context_with_defaults,
                            declared_defaults, LAUNCH_DIR, load_module,
                            load_robot_launch, node_name, node_params, run)
from launch_ros.actions import Node
import pytest

NAV2_SERVERS = ['controller_server', 'planner_server', 'behavior_server',
                'bt_navigator', 'waypoint_follower']


@pytest.fixture
def nav2_launch(monkeypatch):
    return load_module('nav2_launch.py', monkeypatch)


@pytest.fixture
def robot_launch(tmp_path, monkeypatch):
    return load_robot_launch(tmp_path, monkeypatch)


def node_remappings(context, node):
    # launch_ros has no public getter before the node is executed
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
    assert declared_defaults(nav2_launch) == {
        'use_sim_time': 'false',
        'params_file': os.path.join(CONFIG_DIR, 'nav2_params.yaml'),
    }


def test_nav2_launch_starts_servers_and_one_lifecycle_manager(nav2_nodes):
    _, nodes = nav2_nodes

    assert sorted(nodes) == sorted(
        NAV2_SERVERS + ['lifecycle_manager_navigation'])


def test_lifecycle_manager_activates_every_server(nav2_nodes):
    context, nodes = nav2_nodes
    _, values = node_params(context, nodes['lifecycle_manager_navigation'])

    assert values['autostart'] is True
    assert sorted(values['node_names']) == sorted(NAV2_SERVERS)


@pytest.mark.parametrize('server', NAV2_SERVERS)
def test_servers_load_the_params_file(nav2_nodes, server):
    context, nodes = nav2_nodes

    files, values = node_params(context, nodes[server])

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
            assert node_params(context, node)[1]['use_sim_time'] is True


# mobile_robot_launch.py nav argument

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
    assert declared_defaults(robot_launch)['nav'] == 'false'


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
