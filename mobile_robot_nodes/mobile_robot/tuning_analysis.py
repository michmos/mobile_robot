"""
Math behind the wheel_tuning script, kept free of ROS so it can be tested.

All wheel velocities are in rad/s, duties are fractions in [-1, 1], like on
the esp32 (esp32_firmware/inc/MotorControl.hpp).
"""

from collections import namedtuple
import math

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


def mean_velocity(samples):
    """
    Mean velocity from position samples: delta position / delta time.

    Far more accurate than averaging the reported velocities, which are
    quantized to one encoder tick per report (~0.65 rad/s at 50 Hz).

    :param samples: [(time s, position rad)] in time order
    :returns float: rad/s, None with fewer than two samples or no time passed
    """
    if len(samples) < 2:
        return None
    (t_first, pos_first), (t_last, pos_last) = samples[0], samples[-1]
    if not t_last > t_first:
        return None
    return (pos_last - pos_first) / (t_last - t_first)


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
    used = [(abs(p.omega_actual), abs(p.duty)) for p in points
            if not p.stalled and abs(p.duty) < saturation]
    if len(used) < 2:
        return None
    n = len(used)
    mean_x = sum(x for x, _ in used) / n
    mean_y = sum(y for _, y in used) / n
    sxx = sum((x - mean_x) ** 2 for x, _ in used)
    if sxx == 0.0:
        return None
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in used)
    slope = sxy / sxx
    min_duty = mean_y - slope * mean_x
    rms = math.sqrt(
        sum((y - (min_duty + slope * x)) ** 2 for x, y in used) / n)
    return FfFit(min_duty=min_duty, slope=slope, n_points=n, rms_error=rms)


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
