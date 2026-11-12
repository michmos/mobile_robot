"""
Save the map of a running slam_toolbox to <map dir>/<name>/.

Calls /slam_toolbox/serialize_map (pose graph: <name>.posegraph + .data) and
/slam_toolbox/save_map (occupancy grid: <name>.pgm + .yaml, written by
nav2_map_server's map_saver_cli). slam_toolbox writes the files itself, so
run this on the machine slam_toolbox runs on.
"""

import argparse
import os
import sys

from mobile_robot.map_storage import default_map_dir, map_base_path
import rclpy
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_msgs.msg import String

SERVICE_TIMEOUT = 10.0
# map_saver_cli waits for /map and writes the image, slow for a big map
CALL_TIMEOUT = 30.0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='save_map', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('name', help='map name, also the folder name')
    parser.add_argument(
        '--map-dir', default=default_map_dir(),
        help='directory holding the maps (default: $MAP_DIR or ~/maps, '
             'now %(default)s)')
    return parser.parse_args(argv)


def call(node, service_type, name, request):
    """Call a service, return the response or None on timeout."""
    client = node.create_client(service_type, name)
    try:
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            node.get_logger().error(f'{name} not available, is slam_toolbox '
                                    'running?')
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future,
                                         timeout_sec=CALL_TIMEOUT)
        if future.result() is None:
            node.get_logger().error(f'{name}: no response')
        return future.result()
    finally:
        node.destroy_client(client)


def save(node, base_path):
    """Serialize the pose graph and save the grid, True if both worked."""
    serialized = call(node, SerializePoseGraph, '/slam_toolbox/serialize_map',
                      SerializePoseGraph.Request(filename=base_path))
    if serialized is None or \
            serialized.result != SerializePoseGraph.Response.RESULT_SUCCESS:
        node.get_logger().error('serialize_map failed')
        return False
    saved = call(node, SaveMap, '/slam_toolbox/save_map',
                 SaveMap.Request(name=String(data=base_path)))
    if saved is None or saved.result != SaveMap.Response.RESULT_SUCCESS:
        node.get_logger().error('save_map failed')
        return False
    return True


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        base_path = map_base_path(args.name, args.map_dir)
    except ValueError as error:
        print(f'save_map: {error}', file=sys.stderr)
        return 2
    os.makedirs(os.path.dirname(base_path), exist_ok=True)

    rclpy.init()
    node = rclpy.create_node('save_map')
    try:
        ok = save(node, base_path)
        if ok:
            node.get_logger().info(f'saved map to {base_path}.*')
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
