import tempfile
import xml.etree.ElementTree as ET

import yaml

LEFT_WHEEL_JOINT = 'left_wheel_joint'
RIGHT_WHEEL_JOINT = 'right_wheel_joint'
CONTROLLER_NAME = 'mobile_robot_controller'


def find_joint(urdf_root, joint_name):
    joint = urdf_root.find(f"joint[@name='{joint_name}']")
    if joint is None:
        raise ValueError(f"joint '{joint_name}' not found in URDF")
    return joint


def joint_y(joint):
    return float(joint.find('origin').get('xyz').split()[1])


def wheel_radius(urdf_root, joint):
    link_name = joint.find('child').get('link')
    cylinder = urdf_root.find(
        f"link[@name='{link_name}']/collision/geometry/cylinder")
    return float(cylinder.get('radius'))


def kinematics_from_urdf(urdf_xml):
    """Wheel separation (center to center) and radius from the URDF."""
    urdf_root = ET.fromstring(urdf_xml)
    left = find_joint(urdf_root, LEFT_WHEEL_JOINT)
    right = find_joint(urdf_root, RIGHT_WHEEL_JOINT)
    if left.find('parent').get('link') != right.find('parent').get('link'):
        raise ValueError('wheel joints must share the same parent link')
    radius = wheel_radius(urdf_root, left)
    if not abs(radius - wheel_radius(urdf_root, right)) < 1e-9:
        raise ValueError('left and right wheel radius differ')
    return {'wheel_separation': abs(joint_y(left) - joint_y(right)),
            'wheel_radius': radius}


def write_kinematics_params(urdf_xml):
    """Write the URDF-derived controller params to a temporary YAML file."""
    params = {CONTROLLER_NAME: {'ros__parameters':
                                kinematics_from_urdf(urdf_xml)}}
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='mobile_robot_kinematics_', suffix='.yaml',
            delete=False) as file:
        yaml.safe_dump(params, file)
    return file.name
