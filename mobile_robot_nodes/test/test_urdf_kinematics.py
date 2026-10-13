"""Unit tests for mobile_robot.urdf_kinematics."""

import os
import tempfile
import xml.etree.ElementTree as ET

from mobile_robot.urdf_kinematics import (
    find_joint,
    joint_y,
    kinematics_from_urdf,
    wheel_radius,
    write_kinematics_params,
)
import pytest
import xacro
import yaml

XACRO_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir,
    'description', 'mobile_robot.urdf.xacro')


def wheel_link(name, radius):
    return f"""
  <link name="{name}">
    <visual>
      <geometry><cylinder length="0.03" radius="999"/></geometry>
    </visual>
    <collision>
      <geometry><cylinder length="0.03" radius="{radius}"/></geometry>
    </collision>
  </link>"""


def wheel_joint(name, child, y, parent='base_link'):
    return f"""
  <joint name="{name}" type="continuous">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="0 {y} 0" rpy="0 0 0"/>
  </joint>"""


def make_urdf(left_y=0.1, right_y=-0.1, left_radius=0.03,
              right_radius=0.03, left_parent='base_link',
              right_parent='base_link'):
    """Minimal two-wheel URDF; the defaults describe a valid robot."""
    return f"""<robot name="test_robot">
  <link name="base_link"/>
  <link name="other_link"/>
  {wheel_link('left_wheel', left_radius)}
  {wheel_link('right_wheel', right_radius)}
  {wheel_joint('left_wheel_joint', 'left_wheel', left_y, left_parent)}
  {wheel_joint('right_wheel_joint', 'right_wheel', right_y, right_parent)}
</robot>"""


# find_joint

def test_find_joint_returns_named_joint():
    root = ET.fromstring(make_urdf())

    joint = find_joint(root, 'left_wheel_joint')

    assert joint.get('name') == 'left_wheel_joint'


def test_find_joint_raises_on_missing_joint():
    root = ET.fromstring(make_urdf())

    with pytest.raises(ValueError, match='caster_joint'):
        find_joint(root, 'caster_joint')


# joint_y, wheel_radius

def test_joint_y_parses_origin_y():
    root = ET.fromstring(make_urdf(left_y=0.0955))

    assert joint_y(find_joint(root, 'left_wheel_joint')) == pytest.approx(
        0.0955, abs=1e-12)


def test_joint_y_parses_negative_origin_y():
    root = ET.fromstring(make_urdf(right_y=-0.0955))

    assert joint_y(find_joint(root, 'right_wheel_joint')) == pytest.approx(
        -0.0955, abs=1e-12)


def test_wheel_radius_reads_collision_cylinder_of_child_link():
    # The visual cylinder has a bogus radius: the collision geometry is
    # the one that touches the ground.
    root = ET.fromstring(make_urdf(left_radius=0.0335))

    radius = wheel_radius(root, find_joint(root, 'left_wheel_joint'))

    assert radius == pytest.approx(0.0335, abs=1e-12)


# kinematics_from_urdf

def test_kinematics_from_urdf_symmetric_wheels():
    params = kinematics_from_urdf(make_urdf(left_y=0.1, right_y=-0.1,
                                            left_radius=0.03,
                                            right_radius=0.03))

    assert params == {'wheel_separation': pytest.approx(0.2, abs=1e-12),
                      'wheel_radius': pytest.approx(0.03, abs=1e-12)}


@pytest.mark.parametrize('left_y, right_y, separation', [
    # wheels offset from base_link's center still give center-to-center
    (0.15, -0.05, 0.2),
    # left/right swapped in the URDF: separation stays positive
    (-0.1, 0.1, 0.2),
    # degenerate: both wheels on the same spot
    (0.0, 0.0, 0.0),
])
def test_kinematics_from_urdf_separation_is_abs_y_difference(
        left_y, right_y, separation):
    params = kinematics_from_urdf(make_urdf(left_y=left_y, right_y=right_y))

    assert params['wheel_separation'] == pytest.approx(separation, abs=1e-12)


def test_kinematics_from_urdf_raises_on_different_parent_links():
    # y offsets in different frames can't be subtracted.
    urdf = make_urdf(right_parent='other_link')

    with pytest.raises(ValueError, match='parent link'):
        kinematics_from_urdf(urdf)


def test_kinematics_from_urdf_raises_on_wheel_radius_mismatch():
    # The controller has a single radius; a mismatch would skew odometry.
    urdf = make_urdf(left_radius=0.0335, right_radius=0.0336)

    with pytest.raises(ValueError, match='radius'):
        kinematics_from_urdf(urdf)


def test_kinematics_from_urdf_accepts_radius_difference_below_tolerance():
    # Float noise from xacro expressions must not count as a mismatch.
    params = kinematics_from_urdf(make_urdf(left_radius=0.0335,
                                            right_radius=0.0335 + 1e-12))

    assert params['wheel_radius'] == pytest.approx(0.0335, abs=1e-9)


def test_kinematics_from_urdf_raises_on_missing_wheel_joint():
    urdf = '<robot name="no_wheels"><link name="base_link"/></robot>'

    with pytest.raises(ValueError, match='left_wheel_joint'):
        kinematics_from_urdf(urdf)


# write_kinematics_params

def test_write_kinematics_params_writes_controller_yaml(tmp_path,
                                                        monkeypatch):
    # The function writes to the system temp dir; redirect it so the test
    # leaves no file behind.
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))

    path = write_kinematics_params(make_urdf(left_y=0.1, right_y=-0.1,
                                             left_radius=0.03,
                                             right_radius=0.03))

    assert os.path.dirname(path) == str(tmp_path)
    with open(path) as file:
        params = yaml.safe_load(file)
    assert params == {'mobile_robot_controller': {'ros__parameters': {
        'wheel_separation': pytest.approx(0.2, abs=1e-12),
        'wheel_radius': pytest.approx(0.03, abs=1e-12)}}}
    # ROS 2 rejects an int for a double parameter, so the type matters.
    values = params['mobile_robot_controller']['ros__parameters']
    assert all(isinstance(v, float) for v in values.values())


def test_write_kinematics_params_does_not_write_on_invalid_urdf(tmp_path,
                                                                monkeypatch):
    monkeypatch.setattr(tempfile, 'tempdir', str(tmp_path))

    with pytest.raises(ValueError):
        write_kinematics_params(make_urdf(right_radius=0.05))

    assert list(tmp_path.iterdir()) == []


# real robot description

def test_kinematics_from_real_robot_description():
    urdf_xml = xacro.process_file(XACRO_FILE).toxml()

    params = kinematics_from_urdf(urdf_xml)

    # properties.xacro: 2 * (chassis_width/2 + wheel_gap + wheel_length/2)
    #                 = 2 * (0.075 + 0.007 + 0.0135) = 0.191
    assert params['wheel_separation'] == pytest.approx(0.191, abs=1e-9)
    assert params['wheel_radius'] == pytest.approx(0.0335, abs=1e-9)
