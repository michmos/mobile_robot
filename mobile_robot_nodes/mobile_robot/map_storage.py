"""
Location of the saved SLAM maps.

A map <name> lives in <map dir>/<name>/ as
- <name>.posegraph + <name>.data: slam_toolbox's serialized pose graph
  (localization mode, continue mapping)
- <name>.pgm + <name>.yaml: occupancy grid (nav2 map_server)

The map dir is $MAP_DIR, or ~/maps if it is unset.
"""

import os

MAP_DIR_ENV = 'MAP_DIR'
DEFAULT_MAP_DIR = '~/maps'
POSE_GRAPH_SUFFIX = '.posegraph'


def default_map_dir():
    """$MAP_DIR, or ~/maps if unset or empty (not expanded)."""
    return os.environ.get(MAP_DIR_ENV) or DEFAULT_MAP_DIR


def map_base_path(name, map_dir):
    """
    Absolute path of map <name> without file extension.

    slam_toolbox's serialize_map/save_map and map_file_name take this path
    and append the extensions themselves.

    :raises ValueError: name is empty, '.'/'..' or contains a path separator
    """
    if name in ('', '.', '..') or os.sep in name or \
            (os.altsep and os.altsep in name):
        raise ValueError(
            f"map name must be a plain file name, got '{name}'")
    if not map_dir:
        raise ValueError('map dir must not be empty')
    directory = os.path.abspath(os.path.expanduser(map_dir))
    return os.path.join(directory, name, name)
