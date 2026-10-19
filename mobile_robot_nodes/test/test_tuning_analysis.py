"""Unit tests for mobile_robot.tuning_analysis (math of wheel_tuning)."""

import math

from mobile_robot.tuning_analysis import (
    check_step,
    combined_score,
    deadband_bracket,
    error_peaks,
    feedforward_duty,
    FfPoint,
    fit_feedforward,
    fit_needed_duty,
    flag_outliers,
    GridResult,
    iae,
    is_oscillating,
    is_stalled,
    iterate_feedforward,
    joint_velocity_limit,
    mean_velocity,
    needed_duty,
    rank_grid,
    relative_errors,
    select_fit_points,
    smooth,
    step_metrics,
    step_score,
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


# mean_velocity with max_rate


def test_mean_velocity_skips_position_jumps():
    # 2 rad/s, with a jump of the reported position by 100 rad in between
    samples = [(0.0, 0.0), (1.0, 2.0), (1.02, 102.04), (2.02, 104.04)]
    assert mean_velocity(samples, max_rate=40.0) == pytest.approx(
        2.0, abs=1e-6)


def test_mean_velocity_max_rate_keeps_real_motion():
    samples = [(0.0, 0.0), (0.5, 5.0), (1.0, 10.0)]
    assert mean_velocity(samples, max_rate=40.0) == pytest.approx(10.0)


def test_mean_velocity_max_rate_all_jumps_is_none():
    assert mean_velocity([(0.0, 0.0), (0.01, 100.0)], max_rate=40.0) is None


# select_fit_points / flag_outliers

# bench run 1, left wheel forward (--baseline, duty = cmd / 10)
BENCH_DUTIES = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
BENCH_LEFT = [0.0, 2.09, 6.25, 9.57, 11.73, 13.20, 14.16, 14.98, 15.60,
              16.97]


def bench_points(omegas, sign=1.0):
    return [FfPoint(sign * d, sign * w, w < 0.2)
            for d, w in zip(BENCH_DUTIES, omegas)]


def test_select_keeps_only_the_drivable_range():
    used, excluded = select_fit_points(bench_points(BENCH_LEFT), fit_min=1.0,
                                       fit_max=11.0)
    assert sorted(p.duty for p in used) == [0.2, 0.3, 0.4]
    reasons = {round(p.duty, 3): r for p, r in excluded}
    assert reasons[0.1] == 'stalled'
    assert reasons[0.5] == 'above fit range'
    assert reasons[0.9] == 'saturated'


def test_range_fit_matches_the_bench_curve_where_it_is_driven():
    used, _ = select_fit_points(bench_points(BENCH_LEFT), fit_min=1.0,
                                fit_max=11.0)
    fit = fit_feedforward(used)
    assert fit.min_duty == pytest.approx(0.141, abs=0.005)
    assert fit.slope == pytest.approx(0.0266, abs=0.001)


def test_select_near_stall_and_below_fit_min():
    points = [FfPoint(0.15, 0.0, True), FfPoint(0.16, 3.0, False),
              FfPoint(0.2, 0.5, False), FfPoint(0.3, 6.0, False),
              FfPoint(0.4, 9.0, False)]
    _, excluded = select_fit_points(points, fit_min=1.0, fit_max=11.0,
                                    stall_margin=0.025)
    reasons = {p.duty: r for p, r in excluded}
    assert reasons[0.16] == 'near stall'
    assert reasons[0.2] == 'below fit range'


def test_select_judges_directions_separately():
    # reverse stalls up to 0.2, forward only at 0.1
    points = [FfPoint(0.1, 0.0, True), FfPoint(0.2, 3.0, False),
              FfPoint(0.3, 6.0, False), FfPoint(-0.2, 0.0, True),
              FfPoint(-0.21, -0.4, False), FfPoint(-0.3, -6.0, False)]
    used, excluded = select_fit_points(points, fit_min=1.0, fit_max=11.0)
    assert sorted(p.duty for p in used) == [-0.3, 0.2, 0.3]
    assert (FfPoint(-0.21, -0.4, False), 'near stall') in excluded


def test_outlier_in_the_middle_is_flagged_not_its_neighbours():
    duties = [0.2, 0.225, 0.25, 0.275, 0.3]
    omegas = [2.0, 3.0, 0.9, 5.0, 6.0]
    points = [FfPoint(d, w, False) for d, w in zip(duties, omegas)]
    assert flag_outliers(points, 0.4) == {2}


def test_outlier_at_the_deadband_end_is_flagged():
    # bench run 2: the left wheel barely turned at duty 0.2
    points = [FfPoint(0.2, 0.61, False), FfPoint(0.3, 6.47, False),
              FfPoint(0.4, 9.85, False)]
    assert flag_outliers(points, 0.4) == {0}


def test_concave_bench_curve_has_no_outliers():
    points = [FfPoint(d, w, False)
              for d, w in zip(BENCH_DUTIES[1:4], BENCH_LEFT[1:4])]
    assert flag_outliers(points, 0.4) == set()


def test_outliers_need_three_points():
    points = [FfPoint(0.2, 0.1, False), FfPoint(0.3, 6.0, False)]
    assert flag_outliers(points, 0.4) == set()


def test_select_excludes_outliers_from_the_fit():
    points = [FfPoint(0.2, 2.0, False), FfPoint(0.25, 4.0, False),
              FfPoint(0.3, 1.2, False), FfPoint(0.35, 8.0, False),
              FfPoint(0.4, 10.0, False)]
    used, excluded = select_fit_points(points, fit_min=1.0, fit_max=11.0)
    assert (FfPoint(0.3, 1.2, False), 'outlier') in excluded
    assert len(used) == 4


# needed_duty / fit_needed_duty


def test_needed_duty_interpolates_between_the_nearest_speeds():
    points = [FfPoint(0.2, 2.0, False), FfPoint(0.3, 6.0, False),
              FfPoint(-0.4, -10.0, False)]
    assert needed_duty(points, 4.0) == pytest.approx(0.25)
    assert needed_duty(points, -8.0) == pytest.approx(0.35)
    assert needed_duty(points, 6.0) == pytest.approx(0.3)


def test_needed_duty_outside_the_measured_speeds_is_none():
    points = [FfPoint(0.2, 2.0, False), FfPoint(0.3, 6.0, False)]
    assert needed_duty(points, 1.0) is None
    assert needed_duty(points, 7.0) is None


def test_fit_needed_duty_is_exact_on_a_linear_motor():
    points = motor_points(0.14, 0.026, [1.5, 3.3, 5.1, 7.7, 10.4])
    fit = fit_needed_duty(points, [2.0, 4.0, 6.0, 8.0, 10.0])
    assert fit.min_duty == pytest.approx(0.14, abs=1e-9)
    assert fit.slope == pytest.approx(0.026, abs=1e-9)


def test_fit_needed_duty_weights_low_speeds_more():
    # duty over speed bends up (motor curve flattens): the weighted line
    # stays closer to the low speed end than an unweighted one
    omegas = [2.0, 4.0, 6.0, 8.0, 10.0]
    points = [FfPoint(0.14 + 0.02 * w + 0.001 * w * w, w, False)
              for w in [1.0] + omegas + [11.0]]
    weighted = fit_needed_duty(points, omegas)
    plain = fit_feedforward([p for p in points
                             if 2.0 <= p.omega_actual <= 10.0])
    need_at_2 = 0.14 + 0.04 + 0.004
    assert abs(weighted.min_duty + weighted.slope * 2.0 - need_at_2) < abs(
        plain.min_duty + plain.slope * 2.0 - need_at_2)


# relative_errors


def test_relative_errors():
    points = [FfPoint(0.3, 4.4, False), FfPoint(-0.3, -3.0, False),
              FfPoint(0.1, 0.0, True)]
    assert relative_errors([4.0, -4.0, 1.0], points) == pytest.approx(
        [0.1, 0.25, 1.0])


# iterate_feedforward, with a fake plant that maps duty -> wheel speed


def plant_measure(curves):
    """measure() of iterate_feedforward over {wheel: duty -> omega}."""
    def measure(params, omega_cmds):
        out = {}
        for wheel, (min_duty, slope) in params.items():
            out[wheel] = []
            for cmd in omega_cmds:
                duty = feedforward_duty(cmd, slope, min_duty)
                omega = math.copysign(curves[wheel](abs(duty)), duty)
                out[wheel].append(FfPoint(duty, omega, abs(omega) < 0.2))
        return out
    return measure


def linear_motor(min_duty, slope):
    return lambda d: max(0.0, (d - min_duty) / slope)


def concave_motor(d):
    # like the bench: steep above the deadband, flattening out
    return 0.0 if d < 0.15 else 18.0 * (1.0 - math.exp(-(d - 0.15) / 0.25))


def select_range(points):
    return select_fit_points(points, fit_min=1.0, fit_max=11.0)[0]


CMDS = [2.0, 4.0, 6.0, 8.0, 10.0]


def test_iterate_converges_on_a_linear_motor():
    measure = plant_measure({'left': linear_motor(0.14, 0.026)})
    final, rounds, status = iterate_feedforward(
        measure, {'left': (0.05, 0.04)}, CMDS, tolerance=0.05,
        max_rounds=5, select=select_range)
    assert status == {'left': 'converged'}
    assert final['left'] == pytest.approx((0.14, 0.026), abs=1e-3)
    # round 1 measures the start values, round 2 the refit
    assert [r.round for r in rounds] == [1, 2]
    assert rounds[-1].max_error <= 0.05


def test_iterate_already_good_start_needs_one_round():
    measure = plant_measure({'left': linear_motor(0.14, 0.026)})
    final, rounds, status = iterate_feedforward(
        measure, {'left': (0.14, 0.026)}, CMDS, tolerance=0.05,
        max_rounds=5, select=select_range)
    assert status == {'left': 'converged'}
    assert len(rounds) == 1


def concave_baseline():
    """--baseline sweep of the concave motor and its range fit."""
    measure = plant_measure({'left': concave_motor})
    seed = measure({'left': (0.0, 0.1)},
                   [1.5 + 0.25 * i for i in range(15)])['left']
    fit = fit_feedforward(select_range(seed))
    return measure, seed, (fit.min_duty, fit.slope)


def test_iterate_on_a_concave_motor_gets_stuck_and_keeps_the_best():
    measure, seed, start = concave_baseline()
    final, rounds, status = iterate_feedforward(
        measure, {'left': start}, CMDS, tolerance=0.01, max_rounds=10,
        select=select_range, seed={'left': seed})
    assert status['left'] in ('stuck', 'max rounds')
    best_round = min(rounds, key=lambda r: r.max_error)
    assert best_round.max_error > 0.01
    # the result is the verified round with the smallest error
    assert final['left'] == (best_round.min_duty, best_round.slope)
    assert best_round.max_error < rounds[0].max_error


def test_iterate_concave_motor_converges_with_a_looser_tolerance():
    measure, seed, start = concave_baseline()
    _, rounds, status = iterate_feedforward(
        measure, {'left': start}, CMDS, tolerance=0.1, max_rounds=10,
        select=select_range, seed={'left': seed})
    assert status == {'left': 'converged'}
    assert rounds[0].max_error > 0.1


def test_iterate_uses_the_seed_points():
    # a seed sweep around the commanded speeds makes round 2 exact
    seed = motor_points(0.14, 0.026, [1.5, 3.0, 5.0, 7.0, 9.0, 10.5])
    measure = plant_measure({'left': linear_motor(0.14, 0.026)})
    final, rounds, status = iterate_feedforward(
        measure, {'left': (0.05, 0.04)}, CMDS, tolerance=0.001,
        max_rounds=2, select=select_range, seed={'left': seed})
    assert status == {'left': 'converged'}
    assert final['left'] == pytest.approx((0.14, 0.026), abs=1e-9)


def test_iterate_wheels_are_independent():
    measure = plant_measure({'left': linear_motor(0.14, 0.026),
                             'right': linear_motor(0.13, 0.0255)})
    final, rounds, status = iterate_feedforward(
        measure, {'left': (0.14, 0.026), 'right': (0.0, 0.05)}, CMDS,
        tolerance=0.05, max_rounds=5, select=select_range)
    assert status == {'left': 'converged', 'right': 'converged'}
    assert [r.wheel for r in rounds if r.round == 2] == ['right']
    assert final['right'] == pytest.approx((0.13, 0.0255), abs=1e-3)


def test_iterate_without_moving_points_gives_up():
    measure = plant_measure({'left': lambda d: 0.0})
    final, rounds, status = iterate_feedforward(
        measure, {'left': (0.0, 0.01)}, CMDS, tolerance=0.05,
        max_rounds=5, select=select_range)
    assert status == {'left': 'no fit'}
    assert final == {'left': (0.0, 0.01)}


def test_iterate_zero_rounds_measures_nothing():
    def measure(params, omega_cmds):
        raise AssertionError('must not measure')
    final, rounds, status = iterate_feedforward(
        measure, {'left': (0.1, 0.02)}, CMDS, tolerance=0.05,
        max_rounds=0, select=select_range)
    assert rounds == [] and final == {}
    assert status == {'left': 'max rounds'}


# joint_velocity_limit

URDF = """<robot name="r">
  <joint name="left_wheel_joint" type="continuous"/>
  <ros2_control name="x" type="system">
    <joint name="left_wheel_joint">
      <command_interface name="velocity">
        <param name="min" value="-10.0"/>
        <param name="max" value="10.0"/>
      </command_interface>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity">
        <param name="min">-8.0</param>
        <param name="max">12.0</param>
      </command_interface>
    </joint>
  </ros2_control>
</robot>"""


def test_joint_velocity_limit_is_the_smallest_bound():
    assert joint_velocity_limit(
        URDF, ['left_wheel_joint', 'right_wheel_joint']) == 8.0


def test_joint_velocity_limit_single_joint():
    assert joint_velocity_limit(URDF, ['left_wheel_joint']) == 10.0


@pytest.mark.parametrize('urdf, joints', [
    (URDF, ['left_wheel_joint', 'missing_joint']),
    ('<robot name="r"><joint name="left_wheel_joint"/></robot>',
     ['left_wheel_joint']),
    ('not xml', ['left_wheel_joint']),
])
def test_joint_velocity_limit_missing_is_none(urdf, joints):
    assert joint_velocity_limit(urdf, joints) is None


# PI grid: iae, step_score, oscillation, check_step, ranking


def first_order(tau, initial=0.0, target=5.0, duration=3.0, dt=0.02):
    times = [i * dt for i in range(int(duration / dt) + 1)]
    values = [target + (initial - target) * math.exp(-t / tau) for t in times]
    return times, values


def oscillation(amplitude_ratio, target=5.0, duration=6.0, dt=0.02,
                period=0.4, first=2.0):
    """Cosine around target from its peak, amplitude * ratio per half period."""
    times = [i * dt for i in range(int(duration / dt) + 1)]
    values = [target + first * amplitude_ratio ** (t / (period / 2)) *
              math.cos(2 * math.pi * t / period) for t in times]
    return times, values


def test_iae_of_a_constant_error():
    assert iae([0.0, 1.0, 2.0], [3.0, 3.0, 3.0], 5.0) == pytest.approx(4.0)


def test_iae_of_a_first_order_step_is_tau_times_step():
    times, values = first_order(0.2, duration=3.0, dt=0.001)
    assert iae(times, values, 5.0) == pytest.approx(0.2 * 5.0, rel=1e-3)


def test_step_score_normalizes_the_step_size():
    small = step_score(*first_order(0.2, target=2.0), 0.0, 2.0)
    large = step_score(*first_order(0.2, target=8.0), 0.0, 8.0)
    assert small == pytest.approx(large, rel=1e-6)
    assert small == pytest.approx(0.2, rel=0.02)


def test_step_score_faster_is_better():
    assert step_score(*first_order(0.1), 0.0, 5.0) < step_score(
        *first_order(0.4), 0.0, 5.0)


def test_step_score_penalizes_overshoot():
    times = [0.0, 0.1, 0.2, 0.3, 1.0]
    values = [0.0, 6.0, 5.0, 5.0, 5.0]
    without = step_score(times, values, 0.0, 5.0, overshoot_weight=0.0)
    weighted = step_score(times, values, 0.0, 5.0, overshoot_weight=2.0)
    assert weighted - without == pytest.approx(2.0 * 0.2)


def test_step_score_rejects_zero_step():
    with pytest.raises(ValueError):
        step_score([0.0, 1.0], [1.0, 1.0], 1.0, 1.0)


def test_error_peaks_skip_the_rise_and_noise():
    values = [0.0, 3.0, 5.1, 6.0, 5.0, 4.0, 5.0, 5.2, 5.0]
    # rise (error > 0) left out, then -1.0 overshoot, +1.0 undershoot, the
    # 0.2 inside the noise floor ignored
    assert error_peaks(values, 5.0, noise_floor=0.3) == [1.0, 1.0]


def test_growing_oscillation_is_oscillating():
    _, values = oscillation(1.1)
    assert is_oscillating(values, 5.0, noise_floor=0.3)


def test_sustained_oscillation_is_oscillating():
    _, values = oscillation(1.0)
    assert is_oscillating(values, 5.0, noise_floor=0.3)


def test_damped_oscillation_is_not_oscillating():
    _, values = oscillation(0.5)
    assert not is_oscillating(values, 5.0, noise_floor=0.3)


def test_first_order_response_is_not_oscillating():
    _, values = first_order(0.2)
    assert not is_oscillating(values, 5.0, noise_floor=0.3)


def test_noise_around_the_target_is_not_oscillating():
    values = [5.0 + (0.25 if i % 2 else -0.25) for i in range(100)]
    assert not is_oscillating(values, 5.0, noise_floor=0.3)


def test_check_step_oscillating():
    times, values = oscillation(1.1)
    assert check_step(times, values, 0.0, 5.0, 6.0, 0.3) == 'oscillating'


def test_check_step_no_progress_after_half_the_step():
    times = [i * 0.1 for i in range(16)]
    values = [1.0] * 16
    assert check_step(times, values, 0.0, 5.0, 3.0, 0.3) == 'no progress'
    # too early to tell
    assert check_step(times[:10], values[:10], 0.0, 5.0, 3.0, 0.3) is None


def test_check_step_good_response_goes_on():
    times, values = first_order(0.2)
    assert check_step(times, values, 0.0, 5.0, 3.0, 0.3) is None
    assert check_step([], [], 0.0, 5.0, 3.0, 0.3) is None


def test_rank_grid_best_first_aborted_last():
    results = [
        GridResult(0.2, 0.0, {'left': 0.5, 'right': 0.7}, None),
        GridResult(2.0, 4.0, {'left': 0.1}, 'left oscillating'),
        GridResult(0.5, 1.0, {'left': 0.3, 'right': 0.3}, None),
        GridResult(1.0, 1.0, {'left': 0.2, 'right': 0.6}, None),
    ]
    assert [(r.kp, r.ki) for r in rank_grid(results)] == [
        (0.5, 1.0), (1.0, 1.0), (0.2, 0.0), (2.0, 4.0)]
    assert [r.kp for r in rank_grid(results, 'left')] == [1.0, 0.5, 0.2,
                                                          2.0]
    assert [r.kp for r in rank_grid(results, 'right')] == [0.5, 1.0, 0.2,
                                                           2.0]


def test_combined_score():
    assert combined_score(GridResult(1.0, 1.0, {'left': 0.2, 'right': 0.4},
                                     None)) == pytest.approx(0.3)
    assert combined_score(GridResult(1.0, 1.0, {'left': 0.2},
                                     'right no progress')) == math.inf
    assert combined_score(GridResult(1.0, 1.0, {}, None)) == math.inf
