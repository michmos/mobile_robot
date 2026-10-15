"""Smoke tests for the use_mock_hardware switch in the robot description."""

import os
import xml.etree.ElementTree as ET

import pytest
import xacro

XACRO_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir,
    'description', 'mobile_robot.urdf.xacro')
WHEEL_JOINTS = ['left_wheel_joint', 'right_wheel_joint']


def expand(**mappings):
    return ET.fromstring(
        xacro.process_file(XACRO_FILE, mappings=mappings).toxml())


def hardware(urdf_root):
    return urdf_root.find('ros2_control/hardware')


def hardware_params(urdf_root):
    return {p.get('name'): p.text for p in hardware(urdf_root).iter('param')}


def test_default_uses_esp32_hardware():
    # Defaults must keep the robot's behavior: the real plugin on the esp32.
    urdf_root = expand()

    assert hardware(urdf_root).find('plugin').text == \
        'mobile_robot_hardware/MobileRobotHardware'
    assert hardware_params(urdf_root)['serial_port'] == '/dev/esp32'


def test_mock_hardware_false_uses_esp32_hardware():
    urdf_root = expand(use_mock_hardware='false')

    assert hardware(urdf_root).find('plugin').text == \
        'mobile_robot_hardware/MobileRobotHardware'


def test_mock_hardware_uses_generic_system():
    urdf_root = expand(use_mock_hardware='true')

    assert hardware(urdf_root).find('plugin').text == \
        'mock_components/GenericSystem'


def test_mock_hardware_integrates_velocity_into_position():
    # Without calculate_dynamics the mock's position state never moves and
    # odometry stays at the origin.
    params = hardware_params(expand(use_mock_hardware='true'))

    assert params == {'calculate_dynamics': 'true'}


@pytest.mark.parametrize('joint', WHEEL_JOINTS)
@pytest.mark.parametrize('use_mock_hardware', ['false', 'true'])
def test_wheel_interfaces_do_not_depend_on_hardware(joint,
                                                    use_mock_hardware):
    # The controller claims the same interfaces on the robot and the mock.
    urdf_root = expand(use_mock_hardware=use_mock_hardware)

    joint_element = urdf_root.find(f"ros2_control/joint[@name='{joint}']")
    assert [i.get('name') for i in joint_element.findall(
        'command_interface')] == ['velocity']
    assert [i.get('name') for i in joint_element.findall(
        'state_interface')] == ['velocity', 'position']


@pytest.mark.parametrize('joint', WHEEL_JOINTS)
def test_mock_hardware_states_start_at_zero(joint):
    urdf_root = expand(use_mock_hardware='true')

    for state in urdf_root.findall(
            f"ros2_control/joint[@name='{joint}']/state_interface"):
        assert float(state.find(
            "param[@name='initial_value']").text) == pytest.approx(0.0)
