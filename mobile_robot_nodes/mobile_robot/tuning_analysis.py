"""
Math behind the wheel_tuning script, kept free of ROS so it can be tested.

All wheel velocities are in rad/s, duties are fractions in [-1, 1], like on
the esp32 (esp32_firmware/inc/MotorControl.hpp).
"""

from collections import namedtuple
import math
import xml.etree.ElementTree as ElementTree

FfPoint = namedtuple('FfPoint', ['duty', 'omega_actual', 'stalled'])
FfPoint.__doc__ = """One steady state point of the feedforward sweep.

duty: duty the esp32 applied (feedforward only, kp = ki = 0).
omega_actual: measured wheel velocity (rad/s).
stalled: True if the wheel did not turn (or turned the wrong way).
"""

FfFit = namedtuple('FfFit', ['min_duty', 'slope', 'n_points', 'rms_error'])
FfFit.__doc__ = """Least squares fit of |duty| = min_duty + slope * |omega|.

rms_error: rms of the duty residuals of the points used in the fit.
"""

IterRound = namedtuple('IterRound', [
    'round', 'wheel', 'min_duty', 'slope', 'max_error', 'mean_error'])
IterRound.__doc__ = """One verification round of a wheel's feedforward.

min_duty, slope: feedforward in effect during the round.
max_error, mean_error: relative speed errors |actual - cmd| / |cmd| of the
    round's points, a stalled point counts as 1.
"""

StepMetrics = namedtuple('StepMetrics', [
    'rise_time', 'overshoot_percent', 'settling_time', 'steady_state_error'])
StepMetrics.__doc__ = """Step response metrics, times in s from the step.

rise_time: 10 % -> 90 % of the step, None if 90 % was never reached.
overshoot_percent: peak beyond the target, in % of the step size.
settling_time: from which on the response stays inside the band around the
    target, None if it is still outside at the last sample.
steady_state_error: target minus the mean of the last samples.
"""


def wheel_to_linear(omega, wheel_radius):
    """Linear velocity (m/s) that makes both wheels turn at omega (rad/s)."""
    if not wheel_radius > 0.0:
        raise ValueError(f'wheel_radius must be > 0, got {wheel_radius}')
    return omega * wheel_radius


def feedforward_duty(omega_cmd, slope, min_duty):
    """Duty of the esp32's feedforward, see MotorControl::feedforward_()."""
    if omega_cmd == 0.0 or not math.isfinite(omega_cmd):
        return 0.0
    duty = math.copysign(min_duty + slope * abs(omega_cmd), omega_cmd)
    return max(-1.0, min(1.0, duty))


def mean_velocity(samples, max_rate=None):
    """
    Mean velocity from position samples: delta position / delta time.

    Far more accurate than averaging the reported velocities, which are
    quantized to one encoder tick per report (~0.65 rad/s at 50 Hz).

    :param samples: [(time s, position rad)] in time order
    :param max_rate: rad/s, intervals faster than this are a jump of the
        position (not motion) and left out, None keeps all
    :returns float: rad/s, None with fewer than two samples or no time passed
    """
    if len(samples) < 2:
        return None
    if max_rate is None:
        (t_first, pos_first), (t_last, pos_last) = samples[0], samples[-1]
        if not t_last > t_first:
            return None
        return (pos_last - pos_first) / (t_last - t_first)
    distance = 0.0
    duration = 0.0
    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if not dt > 0.0 or abs(p1 - p0) > max_rate * dt:
            continue
        distance += p1 - p0
        duration += dt
    return distance / duration if duration > 0.0 else None


def is_stalled(omega_cmd, omega_actual, threshold):
    """Tell if the wheel stands (|omega| < threshold) or turns the wrong way."""
    if omega_actual is None or abs(omega_actual) < threshold:
        return True
    return math.copysign(1.0, omega_actual) != math.copysign(1.0, omega_cmd)


def deadband_bracket(points):
    """
    Bracket the deadband from a sweep.

    :param points: [FfPoint]
    :returns tuple: (highest |duty| that stalled, lowest |duty| that moved),
        either None if there is no such point
    """
    stalled = [abs(p.duty) for p in points if p.stalled]
    moving = [abs(p.duty) for p in points if not p.stalled]
    return (max(stalled) if stalled else None,
            min(moving) if moving else None)


def fit_feedforward(points, saturation=0.9):
    """
    Fit |duty| = min_duty + slope * |omega_actual| through the moving points.

    Stalled points and points at |duty| >= saturation are left out: neither
    lies on the linear part of the motor curve. Both directions can be mixed,
    the fit uses magnitudes like the firmware.

    :param points: [FfPoint]
    :returns FfFit: None with fewer than two usable points at different
        velocities
    """
    return _line_fit([(abs(p.omega_actual), abs(p.duty)) for p in points
                      if not p.stalled and abs(p.duty) < saturation])


def _line_fit(used, weights=None):
    """Least squares y = min_duty + slope * x through [(x, y)], weighted."""
    if len(used) < 2:
        return None
    n = len(used)
    weights = weights or [1.0] * n
    total = sum(weights)
    mean_x = sum(w * x for (x, _), w in zip(used, weights)) / total
    mean_y = sum(w * y for (_, y), w in zip(used, weights)) / total
    sxx = sum(w * (x - mean_x) ** 2 for (x, _), w in zip(used, weights))
    if sxx == 0.0:
        return None
    sxy = sum(w * (x - mean_x) * (y - mean_y)
              for (x, y), w in zip(used, weights))
    slope = sxy / sxx
    min_duty = mean_y - slope * mean_x
    rms = math.sqrt(
        sum((y - (min_duty + slope * x)) ** 2 for x, y in used) / n)
    return FfFit(min_duty=min_duty, slope=slope, n_points=n, rms_error=rms)


def select_fit_points(points, fit_min, fit_max, saturation=0.9,
                      stall_margin=0.025, outlier=0.4):
    """
    Pick the points of a sweep the feedforward is fitted through.

    The motor curve (speed over duty) is only near linear in part of the
    range: right above the deadband stiction makes the speed erratic, at
    high duty the curve flattens out. The fit is done where the robot
    drives, so the feedforward is right there. Each direction (sign of the
    duty) is judged on its own, the deadband can differ between them.

    :param points: [FfPoint]
    :param fit_min: rad/s, slower points are left out
    :param fit_max: rad/s, faster points are left out (the robot can't be
        commanded faster than the joint limit anyway)
    :param saturation: points at |duty| >= saturation are left out
    :param stall_margin: duty, points less than this above the highest
        stalled duty of their direction are left out
    :param outlier: relative speed deviation from the neighbours above
        which a point is an outlier, see flag_outliers()
    :returns tuple: ([FfPoint] used, [(FfPoint, reason)] excluded)
    """
    used = []
    excluded = []
    for sign in (1.0, -1.0):
        side = [p for p in points if math.copysign(1.0, p.duty) == sign]
        stalled = [abs(p.duty) for p in side if p.stalled]
        stall_edge = max(stalled) + stall_margin if stalled else None
        candidates = []
        for p in side:
            if p.stalled:
                reason = 'stalled'
            elif abs(p.duty) >= saturation:
                reason = 'saturated'
            elif stall_edge is not None and abs(p.duty) < stall_edge:
                reason = 'near stall'
            elif abs(p.omega_actual) < fit_min:
                reason = 'below fit range'
            elif abs(p.omega_actual) > fit_max:
                reason = 'above fit range'
            else:
                candidates.append(p)
                continue
            excluded.append((p, reason))
        outliers = flag_outliers(candidates, outlier)
        for i, p in enumerate(candidates):
            if i in outliers:
                excluded.append((p, 'outlier'))
            else:
                used.append(p)
    return used, excluded


def flag_outliers(points, threshold):
    """
    Find points whose speed doesn't fit their neighbours in duty.

    Each point's speed is predicted by lines through pairs of the up to two
    points on either side of it (interpolated or extrapolated). A point
    deviating by more than threshold (relative) from every prediction is an
    outlier: a good point next to an outlier still has a clean pair. The
    worst one is removed first and the rest judged again. Needs three
    points of one direction.

    :param points: [FfPoint], moving and of one direction
    :returns set: indices into points
    """
    remaining = sorted(range(len(points)), key=lambda i: abs(points[i].duty))
    flagged = set()
    while len(remaining) >= 3:
        worst, worst_dev = None, threshold
        for k, i in enumerate(remaining):
            near = [remaining[j] for j in range(max(0, k - 2),
                                                min(len(remaining), k + 3))
                    if j != k]
            devs = []
            for a_index, a in enumerate(near):
                for b in near[a_index + 1:]:
                    pred = _line_at(points[a], points[b],
                                    abs(points[i].duty))
                    if pred is not None:
                        devs.append(abs(abs(points[i].omega_actual) - pred) /
                                    max(abs(pred), 1e-6))
            if devs and min(devs) > worst_dev:
                worst, worst_dev = i, min(devs)
        if worst is None:
            break
        flagged.add(worst)
        remaining.remove(worst)
    return flagged


def _line_at(p, q, duty):
    """|omega| at |duty| on the line through two points, None if vertical."""
    d0, d1 = abs(p.duty), abs(q.duty)
    if d1 == d0:
        return None
    w0, w1 = abs(p.omega_actual), abs(q.omega_actual)
    return w0 + (w1 - w0) * (duty - d0) / (d1 - d0)


def needed_duty(points, omega):
    """
    Duty the wheel needs for |omega|, interpolated from measured points.

    Uses the nearest measured speeds below and above omega, so the shape of
    the motor curve elsewhere doesn't matter.

    :param points: [FfPoint], moving ones, both directions by magnitude
    :returns float: |duty|, None if omega isn't between two measured speeds
    """
    curve = [(abs(p.omega_actual), abs(p.duty)) for p in points]
    below = [c for c in curve if c[0] <= abs(omega)]
    above = [c for c in curve if c[0] >= abs(omega)]
    if not below or not above:
        return None
    (w0, d0), (w1, d1) = max(below), min(above)
    if w1 == w0:
        return d0
    return d0 + (d1 - d0) * (abs(omega) - w0) / (w1 - w0)


def fit_needed_duty(points, omegas):
    """
    Fit min_duty + slope * |omega| through the needed duty at each omega.

    Weighted with 1 / omega^2, so it evens out the relative speed error,
    which a duty error makes larger at low speed.

    :param points: [FfPoint] the needed duty is interpolated from
    :param omegas: rad/s the feedforward should be right at
    :returns FfFit: None with fewer than two speeds that can be interpolated
    """
    used = []
    for omega in sorted({abs(w) for w in omegas}):
        duty = needed_duty(points, omega)
        if duty is not None:
            used.append((omega, duty))
    return _line_fit(used, [1.0 / omega ** 2 for omega, _ in used])


def relative_errors(omega_cmds, points):
    """
    Relative speed error of each point, |actual - cmd| / |cmd|.

    A stalled point (or one turning the wrong way) counts as 1.

    :param omega_cmds: commanded rad/s, one per point, none 0
    :param points: [FfPoint] in the same order
    """
    return [1.0 if p.stalled else abs(p.omega_actual - cmd) / abs(cmd)
            for cmd, p in zip(omega_cmds, points)]


def iterate_feedforward(measure, start, omega_cmds, tolerance, max_rounds,
                        select, seed=None, min_change=1e-4):
    """
    Verify-and-correct loop of the feedforward.

    Each round measures the wheels with their current feedforward (kp = ki
    = 0), so the speed error is the feedforward's alone. A wheel whose
    largest error is within tolerance is done. For the others the duty
    each commanded speed needs is interpolated from all points measured so
    far (each round adds points right where they are needed) and the
    feedforward refitted through those, see fit_needed_duty().

    :param measure: callable({wheel: (min_duty, slope)}, omega_cmds) ->
        {wheel: [FfPoint]} in the order of omega_cmds
    :param start: {wheel: (min_duty, slope)}, e.g. from the sweep's fit
    :param omega_cmds: commanded rad/s, none 0
    :param select: callable([FfPoint]) -> [FfPoint] usable for the fit,
        see select_fit_points()
    :param seed: {wheel: [FfPoint]} measured before, e.g. the sweep
    :param min_change: a refit that changes min_duty and slope by less is
        stuck, a straight line can't do better over these speeds
    :returns tuple: ({wheel: (min_duty, slope)} best verified feedforward,
        [IterRound], {wheel: status}) with status 'converged', 'stuck',
        'max rounds' or 'no fit'
    """
    params = dict(start)
    best = {}
    collected = {wheel: list((seed or {}).get(wheel, [])) for wheel in start}
    status = {wheel: 'max rounds' for wheel in start}
    active = list(start)
    rounds = []
    for number in range(1, max_rounds + 1):
        if not active:
            break
        measured = measure(dict(params), omega_cmds)
        for wheel in list(active):
            errors = relative_errors(omega_cmds, measured[wheel])
            max_error = max(errors)
            rounds.append(IterRound(number, wheel, params[wheel][0],
                                    params[wheel][1], max_error,
                                    sum(errors) / len(errors)))
            if wheel not in best or max_error < best[wheel][1]:
                best[wheel] = (params[wheel], max_error)
            if max_error <= tolerance:
                status[wheel] = 'converged'
                active.remove(wheel)
                continue
            collected[wheel] += measured[wheel]
            fit = fit_needed_duty(select(collected[wheel]), omega_cmds)
            if fit is None or not fit.slope > 0.0:
                status[wheel] = 'no fit'
                active.remove(wheel)
                continue
            new = (max(0.0, fit.min_duty), fit.slope)
            if (abs(new[0] - params[wheel][0]) < min_change and
                    abs(new[1] - params[wheel][1]) < min_change):
                status[wheel] = 'stuck'
                active.remove(wheel)
                continue
            params[wheel] = new
    return ({wheel: value for wheel, (value, _) in best.items()}, rounds,
            status)


def joint_velocity_limit(urdf, joint_names):
    """
    Velocity command limit of the wheel joints from the robot description.

    Reads <ros2_control><joint><command_interface name="velocity"> and its
    min / max params (ros2_control.xacro).

    :param urdf: robot_description XML
    :returns float: smallest |min| / |max| over the joints, None if a joint
        or its limits are missing or the XML can't be parsed
    """
    try:
        root = ElementTree.fromstring(urdf)
    except ElementTree.ParseError:
        return None
    limits = {}
    for joint in root.iter('joint'):
        name = joint.get('name')
        if name not in joint_names:
            continue
        for interface in joint.findall('command_interface'):
            if interface.get('name') != 'velocity':
                continue
            for param in interface.findall('param'):
                if param.get('name') not in ('min', 'max'):
                    continue
                try:
                    value = abs(float(param.get('value', param.text)))
                except (TypeError, ValueError):
                    continue
                limits[name] = min(limits.get(name, math.inf), value)
    if any(name not in limits for name in joint_names):
        return None
    return min(limits.values())


def smooth(values, window):
    """
    Centered moving average, the window shrinks at the edges.

    Centered so the smoothing doesn't add lag to the rise time.
    """
    if window < 1:
        raise ValueError(f'window must be >= 1, got {window}')
    half = window // 2
    out = []
    for i in range(len(values)):
        part = values[max(0, i - half):i + half + 1]
        out.append(sum(part) / len(part))
    return out


def step_metrics(times, values, initial, target, band_fraction=0.05,
                 band_min=0.0, tail_fraction=0.2):
    """
    Rise time, overshoot, settling time and steady state error of a step.

    Works for steps up and down, everything is measured along the step.

    :param times: s since the step was commanded, in order
    :param values: measured velocity at those times
    :param band_fraction: settling band, fraction of the step size
    :param band_min: lower limit of the settling band (rad/s), so encoder
        quantization noise alone doesn't count as unsettled
    :param tail_fraction: share of the last samples the steady state error
        is taken from
    :returns StepMetrics:
    """
    if len(times) != len(values) or not values:
        raise ValueError('times and values must be non-empty and equal size')
    step = target - initial
    if step == 0.0:
        raise ValueError('initial and target must differ')
    progress = [(v - initial) / step for v in values]

    t10 = next((t for t, p in zip(times, progress) if p >= 0.1), None)
    t90 = next((t for t, p in zip(times, progress) if p >= 0.9), None)
    rise_time = t90 - t10 if t10 is not None and t90 is not None else None

    overshoot = max(0.0, max(progress) - 1.0) * 100.0

    band = max(band_fraction * abs(step), band_min)
    outside = [i for i, v in enumerate(values) if abs(v - target) > band]
    if not outside:
        settling_time = times[0]
    elif outside[-1] + 1 < len(values):
        settling_time = times[outside[-1] + 1]
    else:
        settling_time = None

    tail = values[-max(1, int(len(values) * tail_fraction)):]
    steady_state_error = target - sum(tail) / len(tail)
    return StepMetrics(rise_time=rise_time, overshoot_percent=overshoot,
                       settling_time=settling_time,
                       steady_state_error=steady_state_error)
