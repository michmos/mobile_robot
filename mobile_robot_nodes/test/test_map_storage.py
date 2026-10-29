"""Unit tests for the map location (map_storage) and save_map's arguments."""

import os

from mobile_robot.map_storage import default_map_dir, map_base_path
from mobile_robot.save_map import parse_args
import pytest


def test_default_map_dir_is_home_maps_without_env(monkeypatch):
    monkeypatch.delenv('MAP_DIR', raising=False)

    assert default_map_dir() == '~/maps'


def test_map_dir_env_overrides_default(monkeypatch):
    monkeypatch.setenv('MAP_DIR', '/data/maps')

    assert default_map_dir() == '/data/maps'


def test_empty_map_dir_env_falls_back_to_default(monkeypatch):
    # `MAP_DIR= ros2 launch ...` must not save to the working directory
    monkeypatch.setenv('MAP_DIR', '')

    assert default_map_dir() == '~/maps'


def test_map_base_path_is_name_inside_its_own_folder():
    assert map_base_path('kitchen', '/data/maps') == \
        '/data/maps/kitchen/kitchen'


def test_map_base_path_expands_home(monkeypatch):
    monkeypatch.setenv('HOME', '/home/mypi')

    assert map_base_path('kitchen', '~/maps') == \
        '/home/mypi/maps/kitchen/kitchen'


def test_map_base_path_makes_relative_dir_absolute(tmp_path, monkeypatch):
    # slam_toolbox resolves the path in its own working directory
    monkeypatch.chdir(tmp_path)

    assert map_base_path('kitchen', 'maps') == \
        os.path.join(str(tmp_path), 'maps', 'kitchen', 'kitchen')


@pytest.mark.parametrize('name', ['', '.', '..', 'a/b', '/kitchen',
                                  'kitchen/'])
def test_map_base_path_rejects_names_that_are_not_plain(name):
    with pytest.raises(ValueError):
        map_base_path(name, '/data/maps')


def test_map_base_path_rejects_empty_dir():
    with pytest.raises(ValueError):
        map_base_path('kitchen', '')


def test_save_map_uses_map_dir_env_by_default(monkeypatch):
    monkeypatch.setenv('MAP_DIR', '/data/maps')

    args = parse_args(['kitchen'])

    assert (args.name, args.map_dir) == ('kitchen', '/data/maps')


def test_save_map_dir_option_overrides_env(monkeypatch):
    monkeypatch.setenv('MAP_DIR', '/data/maps')

    args = parse_args(['kitchen', '--map-dir', '/tmp/maps'])

    assert args.map_dir == '/tmp/maps'


def test_save_map_needs_a_name():
    with pytest.raises(SystemExit):
        parse_args([])
