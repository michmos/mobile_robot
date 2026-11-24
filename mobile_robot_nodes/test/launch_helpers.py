"""Helpers for tests that build launch descriptions without starting nodes."""

import importlib.util
import os
import tempfile

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.utilities import evaluate_parameters

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


def load_robot_launch(tmp_path, monkeypatch):
    monkeypatch.delenv('MAP_DIR', raising=False)
    # the launch file writes the kinematics params to the system temp dir
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))
    return load_module('mobile_robot_launch.py', monkeypatch)


def declared_arguments(description):
    return [a for a in description.entities
            if isinstance(a, DeclareLaunchArgument)]


def declared_defaults(module):
    return {a.name: perform_substitutions(LaunchContext(), a.default_value)
            for a in declared_arguments(module.generate_launch_description())}


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


def run(module, **configurations):
    """Actions the launch file's OpaqueFunction starts for the arguments."""
    description = module.generate_launch_description()
    context = context_with_defaults(description, **configurations)
    setup = next(a for a in description.entities
                 if isinstance(a, OpaqueFunction))
    return context, [a for a in setup.execute(context)
                     if a.condition is None or a.condition.evaluate(context)]


# launch_ros has no public getters for these before the node is executed

def node_params(context, node):
    """Param files and the merged dict params of a node."""
    files, values = [], {}
    for param in evaluate_parameters(context, node._Node__parameters):
        if isinstance(param, dict):
            values.update(param)
        else:
            files.append(str(param))
    return files, values


def node_name(node):
    return node._Node__node_name
