"""Unit tests for mobile_robot.tuning_analysis (math of wheel_tuning)."""

import math

from mobile_robot.tuning_analysis import (
    deadband_bracket,
    feedforward_duty,
    FfPoint,
    fit_feedforward,
    is_stalled,
    mean_velocity,
    smooth,
    step_metrics,
    wheel_to_linear,
)
import pytest

# wheel_to_linear


def test_wheel_to_linear_uses_radius():
    assert wheel_to_linear(4.0, 0.0335) == pytest.approx(0.134, abs=1e-9)


def test_wheel_to_linear_keeps_sign():
    assert wheel_to_linear(-10.0, 0.0335) == pytest.approx(-0.335, abs=1e-9)


@pytest.mark.parametrize('radius', [0.0, -0.03, math.nan])
def test_wheel_to_linear_rejects_invalid_radius(radius):
    with pytest.raises(ValueError):
        wheel_to_linear(1.0, radius)


# feedforward_duty, must match MotorControl::feedforward_() on the esp32

@pytest.mark.parametrize('omega, expected', [
    (4.0, 0.15 + 0.1 * 4.0),
    (-4.0, -(0.15 + 0.1 * 4.0)),
    (0.0, 0.0),  # a stop command gives no duty, not min_duty
    (100.0, 1.0),  # clamped
    (-100.0, -1.0),
    (math.nan, 0.0),
    (math.inf, 0.0),
])
def test_feedforward_duty(omega, expected):
    assert feedforward_duty(omega, 0.1, 0.15) == pytest.approx(
        expected, abs=1e-9)


def test_baseline_duty_is_a_tenth_of_the_speed():
    # --baseline relies on this: a command of N rad/s is a duty of N / 10
    assert feedforward_duty(7.0, 0.1, 0.0) == pytest.approx(0.7, abs=1e-9)


# mean_velocity

def test_mean_velocity_is_position_delta_over_time():
    samples = [(10.0, 1.0), (10.02, 1.1), (13.0, 7.0)]
    assert mean_velocity(samples) == pytest.approx(2.0, abs=1e-9)


def test_mean_velocity_negative():
    assert mean_velocity([(0.0, 5.0), (2.0, 1.0)]) == pytest.approx(
        -2.0, abs=1e-9)


@pytest.mark.parametrize('samples', [
    [],
    [(0.0, 1.0)],
    [(1.0, 1.0), (1.0, 2.0)],  # repeated timestamp, no time passed
    [(2.0, 1.0), (1.0, 2.0)],  # out of order
])
def test_mean_velocity_without_time_base_is_none(samples):
    assert mean_velocity(samples) is None


# is_stalled

@pytest.mark.parametrize('cmd, actual, expected', [
    (2.0, 1.5, False),
    (-2.0, -1.5, False),
    (2.0, 0.1, True),
    (2.0, 0.2, False),  # at the threshold counts as moving
    (2.0, -1.0, True),  # wrong direction, e.g. a wrong invert_* setting
    (-2.0, 1.0, True),
    (2.0, None, True),  # nothing measured
])
def test_is_stalled(cmd, actual, expected):
    assert is_stalled(cmd, actual, 0.2) is expected


# deadband_bracket

def test_deadband_bracket():
    points = [FfPoint(0.1, 0.0, True), FfPoint(0.2, 0.05, True),
              FfPoint(0.3, 1.0, False), FfPoint(-0.4, -2.0, False)]
    assert deadband_bracket(points) == (0.2, 0.3)


def test_deadband_bracket_without_stalled_or_moving_points():
    assert deadband_bracket([FfPoint(0.3, 1.0, False)]) == (None, 0.3)
    assert deadband_bracket([FfPoint(0.1, 0.0, True)]) == (0.1, None)
    assert deadband_bracket([]) == (None, None)


# fit_feedforward

def motor_points(min_duty, slope, omegas):
    """Points of an ideal motor: duty = min_duty + slope * omega."""
    return [FfPoint(min_duty + slope * w, w, False) for w in omegas]


def test_fit_recovers_ideal_motor():
    fit = fit_feedforward(motor_points(0.15, 0.08, [1.0, 3.0, 5.0, 7.0]))

    assert fit.min_duty == pytest.approx(0.15, abs=1e-9)
    assert fit.slope == pytest.approx(0.08, abs=1e-9)
    assert fit.n_points == 4
    assert fit.rms_error == pytest.approx(0.0, abs=1e-9)


def test_fit_mixes_directions_by_magnitude():
    # the firmware uses one min_duty/slope for both directions
    points = (motor_points(0.15, 0.08, [2.0, 4.0]) +
              [FfPoint(-p.duty, -p.omega_actual, False)
               for p in motor_points(0.15, 0.08, [3.0, 5.0])])
    fit = fit_feedforward(points)

    assert fit.min_duty == pytest.approx(0.15, abs=1e-9)
    assert fit.slope == pytest.approx(0.08, abs=1e-9)


def test_fit_skips_stalled_and_saturated_points():
    points = motor_points(0.15, 0.08, [2.0, 4.0, 6.0]) + [
        FfPoint(0.1, 0.0, True),  # deadband
        FfPoint(0.95, 6.5, False),  # saturated, below the line
    ]
    fit = fit_feedforward(points, saturation=0.9)

    assert fit.n_points == 3
    assert fit.slope == pytest.approx(0.08, abs=1e-9)


def test_fit_saturation_boundary_is_excluded():
    points = motor_points(0.0, 0.1, [2.0, 4.0]) + [FfPoint(0.9, 20.0, False)]
    assert fit_feedforward(points, saturation=0.9).n_points == 2


def test_fit_reports_noise_as_rms_error():
    points = [FfPoint(0.3, 1.0, False), FfPoint(0.5, 3.0, False),
              FfPoint(0.6, 5.0, False)]
    fit = fit_feedforward(points)

    assert fit.rms_error > 0.0


@pytest.mark.parametrize('points', [
    [],
    [FfPoint(0.3, 2.0, False)],
    [FfPoint(0.3, 2.0, False), FfPoint(0.4, 2.0, False)],  # same speed
    [FfPoint(0.1, 0.0, True), FfPoint(0.2, 0.0, True)],
])
def test_fit_without_enough_points_is_none(points):
    assert fit_feedforward(points) is None


# smooth

def test_smooth_window_one_is_identity():
    assert smooth([1.0, 5.0, 2.0], 1) == [1.0, 5.0, 2.0]


def test_smooth_is_centered_and_shrinks_at_edges():
    assert smooth([0.0, 3.0, 6.0, 9.0], 3) == pytest.approx(
        [1.5, 3.0, 6.0, 7.5])


def test_smooth_empty():
    assert smooth([], 5) == []


def test_smooth_rejects_zero_window():
    with pytest.raises(ValueError):
        smooth([1.0], 0)


# step_metrics

TIMES = [0.1 * i for i in range(11)]


def test_step_metrics_first_order_like_response():
    values = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
    m = step_metrics(TIMES, values, 0.0, 5.0)

    # 10 % (0.5) at 0.1 s, 90 % (4.5) at 0.5 s
    assert m.rise_time == pytest.approx(0.4, abs=1e-9)
    assert m.overshoot_percent == pytest.approx(0.0, abs=1e-9)
    assert m.settling_time == pytest.approx(0.5, abs=1e-9)
    assert m.steady_state_error == pytest.approx(0.0, abs=1e-9)


def test_step_metrics_overshoot_delays_settling():
    values = [0.0, 3.0, 6.0, 7.0, 6.5, 6.1, 6.0, 6.0, 6.0, 6.0, 6.0]
    m = step_metrics(TIMES, values, 0.0, 6.0, band_fraction=0.05)

    assert m.overshoot_percent == pytest.approx(100.0 / 6.0, abs=1e-6)
    # 6.5 is outside the 0.3 band, 6.1 inside
    assert m.settling_time == pytest.approx(0.5, abs=1e-9)


def test_step_metrics_step_down():
    # between speeds, e.g. 6 -> 3 rad/s
    values = [6.0, 5.0, 4.0, 3.0, 2.8, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0]
    m = step_metrics(TIMES, values, 6.0, 3.0)

    assert m.rise_time == pytest.approx(0.2, abs=1e-9)
    assert m.overshoot_percent == pytest.approx(0.2 / 3.0 * 100.0, abs=1e-6)
    assert m.steady_state_error == pytest.approx(0.0, abs=1e-9)


def test_step_metrics_never_reaching_target():
    # pure feedforward that is too weak: steady state error, no 90 %
    values = [0.0, 1.0, 2.0, 3.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0]
    m = step_metrics(TIMES, values, 0.0, 6.0)

    assert m.rise_time is None
    assert m.settling_time is None
    assert m.steady_state_error == pytest.approx(2.0, abs=1e-9)


def test_step_metrics_band_min_absorbs_quantization_noise():
    # +-0.65 rad/s is one encoder tick per 20 ms report
    values = [0.0, 2.0, 3.65, 2.35, 3.0, 3.65, 2.35, 3.0, 3.0, 3.65, 3.0]
    without = step_metrics(TIMES, values, 0.0, 3.0, band_fraction=0.05)
    with_min = step_metrics(TIMES, values, 0.0, 3.0, band_fraction=0.05,
                            band_min=0.7)

    assert without.settling_time == pytest.approx(1.0, abs=1e-9)
    assert with_min.settling_time == pytest.approx(0.2, abs=1e-9)


def test_step_metrics_already_settled_at_first_sample():
    m = step_metrics([0.0, 0.1], [3.0, 3.0], 0.0, 3.0)
    assert m.settling_time == pytest.approx(0.0, abs=1e-9)


def test_step_metrics_rejects_zero_step():
    with pytest.raises(ValueError):
        step_metrics(TIMES, [1.0] * 11, 2.0, 2.0)


@pytest.mark.parametrize('times, values', [([], []), ([0.0], [1.0, 2.0])])
def test_step_metrics_rejects_bad_input(times, values):
    with pytest.raises(ValueError):
        step_metrics(times, values, 0.0, 1.0)
