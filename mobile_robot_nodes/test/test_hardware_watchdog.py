"""Unit tests for the decision logic of the hardware watchdog (#2)."""

from mobile_robot.hardware_watchdog import HardwareStateMonitor, MISSING
import pytest

HW = 'mobile_robot_ros2_control'


@pytest.fixture
def monitor():
    return HardwareStateMonitor()


@pytest.fixture
def armed(monitor):
    """Monitor that has seen the hardware component active."""
    monitor.update({HW: 'active'})
    return monitor


def test_no_components_listed_yet_is_no_failure(monitor):
    # controller_manager answers before it has loaded the description
    update = monitor.update({})

    assert update.failed == {}
    assert update.started == set()


@pytest.mark.parametrize('label', ['unconfigured', 'inactive'])
def test_component_not_active_yet_is_not_watched(monitor, label):
    # startup: the component is configured/activated after the CM is up
    update = monitor.update({HW: label})

    assert update.failed == {}
    assert monitor.watched == frozenset()


def test_becoming_active_starts_watching(monitor):
    monitor.update({HW: 'unconfigured'})

    update = monitor.update({HW: 'active'})

    assert update.started == {HW}
    assert update.failed == {}
    assert monitor.watched == {HW}


def test_staying_active_reports_nothing(armed):
    # normal operation, also while a live parameter change runs the esp32
    # handshake: the component stays active until the handshake fails
    update = armed.update({HW: 'active'})

    assert update.started == set()
    assert update.failed == {}


@pytest.mark.parametrize('label', ['inactive', 'unconfigured', 'finalized'])
def test_leaving_active_is_a_failure(armed, label):
    # read() returning ERROR ends in 'unconfigured' (on_error succeeded) or
    # 'finalized'; a manual deactivation in 'inactive'
    update = armed.update({HW: label})

    assert update.failed == {HW: label}


def test_watched_component_no_longer_listed_is_a_failure(armed):
    update = armed.update({})

    assert update.failed == {HW: MISSING}


def test_failure_is_reported_again_while_not_active(armed):
    # the caller exits on the first report, but the monitor stays consistent
    armed.update({HW: 'unconfigured'})

    update = armed.update({HW: 'unconfigured'})

    assert update.failed == {HW: 'unconfigured'}


def test_only_the_component_that_left_active_fails(monitor):
    monitor.update({'a': 'active', 'b': 'active'})

    update = monitor.update({'a': 'active', 'b': 'inactive'})

    assert update.failed == {'b': 'inactive'}


def test_component_that_never_became_active_is_ignored(monitor):
    # e.g. a component deliberately left inactive by its initial state
    monitor.update({'a': 'active', 'b': 'inactive'})

    update = monitor.update({'a': 'active', 'b': 'unconfigured'})

    assert update.failed == {}
    assert monitor.watched == {'a'}


def test_late_component_is_watched_once_active(monitor):
    monitor.update({'a': 'active'})

    update = monitor.update({'a': 'active', 'b': 'active'})

    assert update.started == {'b'}
    assert monitor.watched == {'a', 'b'}
