"""
Bench / floor tuning of the wheel velocity control (feedforward + PI).

Drives both wheels straight through twist_mux and measures them on
/joint_states. Speeds are given in wheel rad/s, the unit the esp32 controls
in; the script converts them to linear.x (m/s) with the wheel radius.

Modes (--mode):
  ff   sweeps a list of wheel speeds with kp = ki = 0, so the duty of every
       point is known (feedforward only), and fits min_duty and slope per
       wheel through the points between --fit-min and --fit-max rad/s.
       --baseline starts from min_duty = 0, slope = 0.1, then a command of
       N rad/s is a duty of N / 10. --iterate N then applies the fit,
       measures the speed error and refits until it is within --tolerance.
  pid  runs step responses with the feedforward in effect and the given
       kp / ki / integral_limit, and reports rise time, overshoot, settling
       time and steady state error per wheel. Lists of --kp / --ki are a
       grid search: every combination is run and scored (IAE / step size +
       weighted overshoot), oscillating or stuck ones are aborted, and the
       ranking suggests the best. Start coarse, then refine around it.
  all  ff, then pid with the fitted feedforward.

Every parameter change starts a handshake with the esp32 that brakes the
wheels (see OVERVIEW.md, "Live tuning"), the script waits for it before
commanding. All parameters are restored on exit; --apply keeps the fitted
feedforward and the given (or the best of a grid) kp / ki / integral_limit.
"""

import argparse
import csv
import datetime
import math
import sys
import time

from geometry_msgs.msg import TwistStamped
from mobile_robot.tuning_analysis import (
    check_step,
    combined_score,
    deadband_bracket,
    feedforward_duty,
    FfPoint,
    fit_feedforward,
    GridResult,
    is_stalled,
    iterate_feedforward,
    joint_velocity_limit,
    mean_velocity,
    rank_grid,
    select_fit_points,
    smooth,
    step_metrics,
    step_score,
    wheel_to_linear,
)
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

WHEELS = ('left', 'right')
TUNING_PARAMS = ('pid_kp', 'pid_ki', 'pid_integral_limit',
                 'ff_slope_left', 'ff_slope_right',
                 'ff_min_duty_left', 'ff_min_duty_right')
# lowercased <ros2_control name=...> of ros2_control.xacro, the hardware
# component's node is named after it
DEFAULT_PARAM_NODE = '/mobile_robot_ros2_control'
CONTROLLER_NODE = '/mobile_robot_controller'
# holds robot_description, the joint limits are read from it
DESCRIPTION_NODE = '/robot_state_publisher'
# properties.xacro, used when the controller can't be asked
DEFAULT_WHEEL_RADIUS = 0.0335
# command_interface max of the wheel joints in ros2_control.xacro, used
# when the robot description can't be read
JOINT_VELOCITY_LIMIT = 10.0
# --baseline sweep: duty 0.15 .. 0.5 in 0.025 steps, the part of the motor
# curve the robot drives in (0.5 is already faster than the joint limit)
BASELINE_SPEEDS = [1.5 + 0.25 * i for i in range(15)]
# without --baseline the feedforward is roughly tuned, speed ~ command
DEFAULT_SPEEDS = [float(s) for s in range(1, 11)]
# a faster position change is a jump of the reported position, not motion
MAX_PLAUSIBLE_RATE = 40.0
# twist_mux and the controller drop a command after 0.5 s
COMMAND_RATE_HZ = 20.0
SERVICE_TIMEOUT = 5.0


class TuningError(Exception):
    """A step of the tuning run failed, the message says which."""


class WheelTuning(Node):
    """Publishes the commands, records /joint_states, sets parameters."""

    def __init__(self, args):
        super().__init__('wheel_tuning')
        self._args = args
        self._joint_names = {'left': args.left_joint,
                             'right': args.right_joint}
        self._linear = 0.0
        self._last_joint_state = None
        # {wheel: [(t, position, velocity)]} while recording, else None
        self._recording = None

        self._cmd_pub = self.create_publisher(TwistStamped, args.topic, 10)
        self._joint_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 50)
        self._cmd_timer = self.create_timer(
            1.0 / COMMAND_RATE_HZ, self._publish_command)

    # ---- commands and measurements ----

    def command(self, linear):
        """Drive straight at linear (m/s), sent now and at COMMAND_RATE_HZ."""
        self._linear = linear
        self._publish_command()

    def _publish_command(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self._linear
        self._cmd_pub.publish(msg)

    def _on_joint_state(self, msg):
        self._last_joint_state = msg
        if self._recording is None:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9
        for wheel, name in self._joint_names.items():
            if name not in msg.name:
                continue
            i = msg.name.index(name)
            position = msg.position[i] if i < len(msg.position) else math.nan
            velocity = msg.velocity[i] if i < len(msg.velocity) else math.nan
            self._recording[wheel].append((t, position, velocity))

    def spin_for(self, duration):
        """Process callbacks (commands, joint states) for duration s."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def record(self, duration):
        """
        Spin for duration s and record the wheel joint states.

        :returns dict: {wheel: [(t, position, velocity)]}, t in s since the
            start of the recording
        """
        return self.record_checked(duration, None)[0]

    def record_checked(self, duration, check, check_every=0.25):
        """
        Like record(), but stop early when check says so.

        :param check: callable({wheel: [(t, position, velocity)]}) -> reason
            or None, called every check_every s with the samples so far
        :returns tuple: (samples as record(), reason or None)
        """
        start = self.get_clock().now().nanoseconds * 1e-9
        self._recording = {wheel: [] for wheel in WHEELS}

        def samples():
            return {wheel: [(t - start, p, v) for t, p, v in recorded]
                    for wheel, recorded in self._recording.items()}
        try:
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                self.spin_for(min(check_every,
                                  deadline - time.monotonic()))
                if check is not None:
                    reason = check(samples())
                    if reason is not None:
                        return samples(), reason
            return samples(), None
        finally:
            self._recording = None

    def wait_for_joint_states(self, timeout):
        deadline = time.monotonic() + timeout
        while self._last_joint_state is None:
            if time.monotonic() > deadline:
                raise TuningError('no /joint_states received, is the robot '
                                  'launch running?')
            rclpy.spin_once(self, timeout_sec=0.05)
        missing = [n for n in self._joint_names.values()
                   if n not in self._last_joint_state.name]
        if missing:
            raise TuningError(f'joints {missing} not in /joint_states')

    # ---- parameters ----

    def _call(self, service_type, service, request, timeout=SERVICE_TIMEOUT):
        client = self.create_client(service_type, service)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(request)
            deadline = time.monotonic() + timeout
            while not future.done():
                if time.monotonic() > deadline:
                    client.remove_pending_request(future)
                    return None
                rclpy.spin_once(self, timeout_sec=0.05)
            return future.result()
        finally:
            self.destroy_client(client)

    def get_params(self, node, names, timeout=SERVICE_TIMEOUT):
        """
        Read parameters of another node.

        :returns dict: {name: value}, None if the node can't be asked or one
            of them isn't declared
        """
        response = self._call(GetParameters, f'{node}/get_parameters',
                              GetParameters.Request(names=list(names)),
                              timeout)
        if response is None or len(response.values) != len(names):
            return None
        if any(v.type == ParameterType.PARAMETER_NOT_SET
               for v in response.values):
            return None
        return {name: parameter_value_to_python(value)
                for name, value in zip(names, response.values)}

    def set_params(self, node, values):
        """Set all values in one call, so the esp32 gets one handshake."""
        request = SetParametersAtomically.Request(parameters=[
            Parameter(name, Parameter.Type.DOUBLE,
                      float(value)).to_parameter_msg()
            for name, value in values.items()])
        response = self._call(SetParametersAtomically,
                              f'{node}/set_parameters_atomically', request)
        if response is None:
            raise TuningError(f'could not reach {node} to set parameters')
        if not response.result.successful:
            raise TuningError(
                f'{node} rejected {values}: {response.result.reason}')

    def find_param_node(self, preferred):
        """
        Find the node that owns the tuning parameters.

        That is the hardware component's node. Tries preferred first, then
        asks every node.
        """
        if self.get_params(preferred, TUNING_PARAMS[:1]) is not None:
            return preferred
        for name, namespace in self.get_node_names_and_namespaces():
            node = f"{namespace.rstrip('/')}/{name}"
            if node == preferred or name == self.get_name():
                continue
            if self.get_params(node, TUNING_PARAMS[:1], timeout=1.0):
                return node
        raise TuningError(
            f'no node with the parameter {TUNING_PARAMS[0]} found '
            f'(tried {preferred} first), pass --param-node')


class TuningRun:
    """One run of the script: the sequence of the selected modes."""

    def __init__(self, node, args):
        self._node = node
        self._args = args
        self._param_node = None
        self._original = None
        self._current = None
        self._radius = None
        self._fit_max = None
        # the first point after a handshake gets a longer settle
        self._fresh_handshake = False
        self._sweep_points = {}
        self.fits = {}
        self.ff_rows = []
        self.pid_rows = []
        self.pid_metrics = []
        # [GridResult] per kp / ki combination, best_pid the best of >= 2
        self.grid = []
        self.best_pid = None

    def setup(self):
        args = self._args
        self._node.wait_for_joint_states(timeout=5.0)
        self._radius = args.wheel_radius
        if self._radius is None:
            got = self._node.get_params(CONTROLLER_NODE, ['wheel_radius'])
            self._radius = (got['wheel_radius'] if got
                            else DEFAULT_WHEEL_RADIUS)
            if not got:
                self._log(f'could not read wheel_radius from '
                          f'{CONTROLLER_NODE}, using {self._radius} m')
        self._param_node = self._node.find_param_node(args.param_node)
        self._original = self._node.get_params(self._param_node,
                                               TUNING_PARAMS)
        if self._original is None:
            raise TuningError(f'could not read the parameters of '
                              f'{self._param_node}')
        self._current = dict(self._original)
        self._fit_max = args.fit_max
        if self._fit_max is None:
            self._fit_max = self._joint_limit() + args.fit_margin
        self._log(f'parameter node {self._param_node}, wheel radius '
                  f'{self._radius} m, fitting {args.fit_min} .. '
                  f'{self._fit_max} rad/s')
        self._log('commands go to ' + args.topic + ', a higher priority '
                  'twist_mux input (joystick, /cmd_vel_joy) overrides them, '
                  'keep it idle')

    def _joint_limit(self):
        got = self._node.get_params(DESCRIPTION_NODE, ['robot_description'])
        limit = None
        if got:
            limit = joint_velocity_limit(
                got['robot_description'],
                [self._args.left_joint, self._args.right_joint])
        if limit is None:
            self._log(f'could not read the joint velocity limit from '
                      f'{DESCRIPTION_NODE}, using {JOINT_VELOCITY_LIMIT} '
                      'rad/s')
            return JOINT_VELOCITY_LIMIT
        return limit

    # ---- feedforward ----

    def run_ff(self):
        args = self._args
        target = {'pid_kp': 0.0, 'pid_ki': 0.0}
        if args.baseline:
            target.update({'ff_slope_left': 0.1, 'ff_slope_right': 0.1,
                           'ff_min_duty_left': 0.0,
                           'ff_min_duty_right': 0.0})
        self._apply(target)

        points = self._sweep(args.speeds, round_number=0)
        self._sweep_points = {wheel: [p for d in points[wheel].values()
                                      for p in d] for wheel in WHEELS}
        self._report_ff(points)
        if args.iterate > 0:
            self._iterate()

    def _sweep(self, speeds, round_number):
        """
        Measure every speed in every direction with the current params.

        :returns dict: {wheel: {direction: [FfPoint]}} in sweep order
        """
        args = self._args
        points = {wheel: {d: [] for d in self._directions()}
                  for wheel in WHEELS}
        for direction in self._directions():
            sign = 1.0 if direction == 'forward' else -1.0
            for omega in speeds:
                omega_cmd = sign * omega
                self._node.command(wheel_to_linear(omega_cmd, self._radius))
                settle = args.settle
                if self._fresh_handshake:
                    # the first point after a handshake was off before
                    settle += args.first_settle
                    self._fresh_handshake = False
                self._node.spin_for(settle)
                samples = self._node.record(args.measure)
                line = [f'{direction:7} cmd {omega_cmd:6.2f} rad/s']
                for wheel in WHEELS:
                    point, actual = self._measure_point(
                        wheel, omega_cmd, samples[wheel])
                    points[wheel][direction].append(point)
                    self.ff_rows.append({
                        'mode': 'ff', 'round': round_number, 'wheel': wheel,
                        'direction': direction, 'omega_cmd': omega_cmd,
                        'duty': point.duty, 'omega_actual': actual,
                        'stalled': point.stalled})
                    shown = 'n/a' if actual is None else f'{actual:6.2f}'
                    line.append(f'{wheel} duty {point.duty:5.3f} -> {shown} '
                                'rad/s' +
                                (' (stalled)' if point.stalled else ''))
                self._log(' | '.join(line))
            self._stop()
        return points

    def _measure_point(self, wheel, omega_cmd, samples):
        args = self._args
        duty = feedforward_duty(
            omega_cmd, self._current[f'ff_slope_{wheel}'],
            self._current[f'ff_min_duty_{wheel}'])
        actual = mean_velocity([(t, p) for t, p, _ in samples],
                               max_rate=MAX_PLAUSIBLE_RATE)
        stalled = is_stalled(omega_cmd, actual, args.stall_threshold)
        if (actual is not None and abs(actual) >= args.stall_threshold and
                math.copysign(1.0, actual) != math.copysign(1.0, omega_cmd)):
            self._log(f'{wheel} wheel turns the wrong way, check '
                      f'invert_motor_{wheel} / invert_encoder_{wheel}')
        return (FfPoint(duty, actual if actual is not None else 0.0,
                        stalled), actual)

    def _select(self, points):
        args = self._args
        return select_fit_points(
            points, args.fit_min, self._fit_max, args.saturation,
            args.stall_margin, args.outlier)

    def _iterate(self):
        args = self._args
        missing = [wheel for wheel in WHEELS if wheel not in self.fits]
        if missing:
            print(f'\nno fit for {missing}, skipping --iterate')
            return
        start = {wheel: (self.fits[wheel][f'ff_min_duty_{wheel}'],
                         self.fits[wheel][f'ff_slope_{wheel}'])
                 for wheel in WHEELS}
        # 1 rad/s is right above the deadband, a few % error there is
        # stiction, not the feedforward
        speeds = args.verify_speeds or [
            s for s in DEFAULT_SPEEDS
            if max(args.fit_min, 2.0) <= s <=
            min(self._fit_max, JOINT_VELOCITY_LIMIT)]
        sign_speeds = [(1.0 if d == 'forward' else -1.0) * s
                       for d in self._directions() for s in speeds]
        round_counter = [0]

        def measure(params, _omega_cmds):
            round_counter[0] += 1
            values = {}
            for wheel, (min_duty, slope) in params.items():
                values[f'ff_min_duty_{wheel}'] = round(min_duty, 4)
                values[f'ff_slope_{wheel}'] = round(slope, 4)
            self._apply(values)
            self._log(f'iteration round {round_counter[0]}: ' + ', '.join(
                f'{k}={v}' for k, v in values.items()))
            swept = self._sweep(speeds, round_number=round_counter[0])
            return {wheel: [p for d in self._directions()
                            for p in swept[wheel][d]]
                    for wheel in WHEELS}

        final, rounds, status = iterate_feedforward(
            measure, start, sign_speeds, args.tolerance, args.iterate,
            lambda points: self._select(points)[0], seed=self._sweep_points)

        print('\n==== feedforward iteration ====')
        print('round wheel  min_duty   slope    max error  mean error')
        for r in rounds:
            print(f'{r.round:5} {r.wheel:5}  {r.min_duty:8.4f}  '
                  f'{r.slope:7.4f}  {r.max_error * 100:8.1f} %  '
                  f'{r.mean_error * 100:8.1f} %')
        for wheel in WHEELS:
            min_duty, slope = final[wheel]
            print(f'{wheel}: {status[wheel]}, best min_duty {min_duty:.4f}, '
                  f'slope {slope:.4f}')
            if status[wheel] != 'converged':
                print(f'  the error stays above {args.tolerance * 100:.0f} %:'
                      ' a straight line does not fit the motor curve over '
                      'these speeds. A smaller --fit-max (and '
                      '--verify-speeds) tunes the range the robot drives in '
                      'most, the PI corrects the rest')
            self.fits[wheel] = {f'ff_min_duty_{wheel}': round(min_duty, 4),
                                f'ff_slope_{wheel}': round(slope, 4)}
        print('\nsuggested:')
        for values in self.fits.values():
            for name, value in values.items():
                print(f'  ros2 param set {self._param_node} {name} {value}')

    def _directions(self):
        return {'forward': ['forward'], 'reverse': ['reverse'],
                'both': ['forward', 'reverse']}[self._args.direction]

    def _report_ff(self, points):
        print('\n==== feedforward ====')
        for wheel in WHEELS:
            all_points = [p for d in points[wheel].values() for p in d]
            stalled, moving = deadband_bracket(all_points)
            print(f'{wheel}: deadband between duty '
                  f'{_fmt(stalled)} (highest stalled) and {_fmt(moving)} '
                  '(lowest moving). Sweeping downwards (--speeds 3 2.5 2 '
                  '...) finds where a turning wheel stalls, a little above '
                  'that is a good min_duty.')
            used, excluded = self._select(all_points)
            for point, reason in excluded:
                if reason != 'stalled':
                    print(f'  not fitted ({reason}): duty {point.duty:6.3f}'
                          f' -> {point.omega_actual:6.2f} rad/s')
            for direction, dir_points in points[wheel].items():
                fit = fit_feedforward(self._select(dir_points)[0])
                print(f'  {direction:7}: {_fmt_fit(fit)}')
            fit = fit_feedforward(used)
            if len(points[wheel]) > 1:
                print(f'  combined: {_fmt_fit(fit)}')
            if fit is None or not fit.slope > 0.0:
                print('  no usable fit, needs two moving points in the fit '
                      'range at different speeds')
                continue
            min_duty = fit.min_duty
            if min_duty < 0.0:
                print(f'  fitted min_duty {min_duty:.4f} < 0, using 0')
                min_duty = 0.0
            if min_duty >= 1.0:
                print(f'  fitted min_duty {min_duty:.4f} >= 1, ignored')
                continue
            self.fits[wheel] = {f'ff_min_duty_{wheel}': round(min_duty, 4),
                                f'ff_slope_{wheel}': round(fit.slope, 4)}
        print('\nsuggested:')
        for values in self.fits.values():
            for name, value in values.items():
                print(f'  ros2 param set {self._param_node} {name} {value}')

    # ---- PI ----

    def run_pid(self, use_fit):
        args = self._args
        base = {'pid_integral_limit': (args.integral_limit
                                       if args.integral_limit is not None
                                       else self._original[
                                           'pid_integral_limit'])}
        if use_fit:
            for values in self.fits.values():
                base.update(values)
        # after ff mode kp/ki are 0, back to the original unless given
        kps = args.kp or [self._original['pid_kp']]
        kis = args.ki or [self._original['pid_ki']]
        combos = [(kp, ki) for kp in kps for ki in kis]
        for number, (kp, ki) in enumerate(combos, 1):
            if len(combos) > 1:
                self._log(f'---- combination {number}/{len(combos)}: kp {kp} '
                          f'ki {ki} ----')
            self.grid.append(self._run_steps(dict(base, pid_kp=kp,
                                                  pid_ki=ki)))
        self._report_pid()

    def _run_steps(self, gains):
        """Run the step sequence with gains, return its GridResult."""
        args = self._args
        self._apply(gains)
        kp, ki = self._current['pid_kp'], self._current['pid_ki']
        self._log('PI: kp {pid_kp} ki {pid_ki} integral_limit '
                  '{pid_integral_limit}, ff left {ff_min_duty_left} + '
                  '{ff_slope_left}/rad/s, right {ff_min_duty_right} + '
                  '{ff_slope_right}/rad/s'.format(**self._current))

        self._node.spin_for(1.0)
        previous = 0.0
        scores = {wheel: [] for wheel in WHEELS}
        reason = None
        for segment, omega in enumerate(args.steps):
            self._node.command(wheel_to_linear(omega, self._radius))
            judged = omega != previous and omega != 0.0
            check = (self._step_check(previous, omega) if judged else None)
            samples, abort = self._node.record_checked(args.step_duration,
                                                       check)
            for wheel in WHEELS:
                times = [t for t, _, _ in samples[wheel]]
                raw = [v for _, _, v in samples[wheel]]
                smoothed = smooth(raw, args.smooth) if raw else []
                for t, v, sm in zip(times, raw, smoothed):
                    self.pid_rows.append({
                        'mode': 'pid', 'kp': kp, 'ki': ki, 'wheel': wheel,
                        'segment': segment, 'omega_cmd': omega, 't': t,
                        'velocity': v, 'velocity_smoothed': sm})
                # a step to 0 brakes on the esp32, nothing to judge
                if not judged or not smoothed or abort:
                    continue
                metrics = step_metrics(
                    times, smoothed, previous, omega,
                    band_fraction=args.settle_band,
                    band_min=args.settle_band_min)
                self.pid_metrics.append((kp, ki, segment, wheel, previous,
                                         omega, metrics))
                scores[wheel].append(step_score(
                    times, smoothed, previous, omega,
                    overshoot_weight=args.overshoot_weight))
            if abort:
                reason = abort
                self._log(f'kp {kp} ki {ki}: {abort}, stopping the wheels '
                          'and moving on')
                break
            previous = omega
        self._stop()
        return GridResult(kp, ki, {wheel: sum(v) / len(v)
                                   for wheel, v in scores.items() if v},
                          reason)

    def _step_check(self, initial, target):
        args = self._args

        def check(samples):
            for wheel in WHEELS:
                times = [t for t, _, _ in samples[wheel]]
                raw = [v for _, _, v in samples[wheel]]
                if not raw:
                    continue
                reason = check_step(times, smooth(raw, args.smooth), initial,
                                    target, args.step_duration,
                                    args.settle_band_min)
                if reason is not None:
                    return f'{wheel} {reason}'
            return None
        return check

    def _report_pid(self):
        print('\n==== PI step responses ====')
        print('   kp     ki  seg wheel  step (rad/s)    rise (s)  overshoot  '
              'settle (s)  ss error (rad/s)')
        for kp, ki, segment, wheel, start, target, m in self.pid_metrics:
            print(f'{kp:5.2f} {ki:6.2f}  {segment:3} {wheel:5}  '
                  f'{start:5.2f} -> {target:5.2f}  '
                  f'{_fmt(m.rise_time, 3):>8}  '
                  f'{m.overshoot_percent:7.1f} %  '
                  f'{_fmt(m.settling_time, 3):>10}  '
                  f'{m.steady_state_error:8.3f}')
        for result in self.grid:
            if result.reason is not None:
                print(f'kp {result.kp} ki {result.ki}: aborted, '
                      f'{result.reason}')
        if len(self.grid) < 2:
            return
        print('\n==== PI grid, best first (score: IAE / step size + '
              f'{self._args.overshoot_weight} x overshoot, lower is better) '
              '====')
        for wheel in (None,) + WHEELS:
            print(f'{wheel or "combined"}:')
            for rank, result in enumerate(rank_grid(self.grid, wheel), 1):
                score = (combined_score(result) if wheel is None
                         else result.scores.get(wheel))
                shown = (result.reason if result.reason is not None
                         else _fmt(score, 3))
                print(f'  {rank:2}. kp {result.kp:6.3f} ki {result.ki:6.3f}  '
                      f'{shown}')
        best = rank_grid(self.grid)[0]
        if best.reason is not None:
            print('\nevery combination was aborted, try smaller gains')
            return
        self.best_pid = {'pid_kp': best.kp, 'pid_ki': best.ki}
        print('\nsuggested (best combined, both wheels share the gains):')
        for name, value in self.best_pid.items():
            print(f'  ros2 param set {self._param_node} {name} {value}')
        print('refine with a finer grid around it, e.g. --kp '
              f'{_fmt(best.kp * 0.7, 2)} {_fmt(best.kp, 2)} '
              f'{_fmt(best.kp * 1.4, 2)} --ki {_fmt(best.ki * 0.7, 2)} '
              f'{_fmt(best.ki, 2)} {_fmt(best.ki * 1.4, 2)}')

    # ---- shared ----

    def _apply(self, values):
        changed = {k: v for k, v in values.items()
                   if self._current.get(k) != v}
        if not changed:
            return
        self._node.command(0.0)
        self._log(f'setting {changed}, waiting for the handshake')
        self._node.set_params(self._param_node, changed)
        self._current.update(changed)
        self._node.spin_for(self._args.handshake_wait)
        self._fresh_handshake = True

    def _stop(self):
        self._node.command(0.0)
        self._node.spin_for(1.0)

    def finish(self):
        """Stop, then restore the parameters (or keep --apply ones)."""
        self._node.command(0.0)
        self._node.spin_for(0.2)
        if self._original is None:
            return
        final = dict(self._original)
        if self._args.apply:
            for values in self.fits.values():
                final.update(values)
            if self.best_pid:
                final.update(self.best_pid)
            for name, values in (('pid_kp', self._args.kp),
                                 ('pid_ki', self._args.ki)):
                if values is not None and len(values) == 1:
                    final[name] = values[0]
            if self._args.integral_limit is not None:
                final['pid_integral_limit'] = self._args.integral_limit
        self._apply(final)
        self._log('parameters now: ' + ', '.join(
            f'{k}={v}' for k, v in self._current.items()))

    def write_csv(self, path):
        rows = self.ff_rows + self.pid_rows
        if not rows:
            return
        fields = ['mode', 'round', 'kp', 'ki', 'wheel', 'direction',
                  'segment', 'omega_cmd',
                  'duty', 'omega_actual', 'stalled', 't', 'velocity',
                  'velocity_smoothed']
        with open(path, 'w', newline='') as file:
            file.write(f'# wheel_tuning {datetime.datetime.now().isoformat()}'
                       '\n')
            file.write('# battery_voltage='
                       f'{_fmt(self._args.battery_voltage)}\n')
            file.write(f'# note={self._args.note}\n')
            file.write(f'# wheel_radius={self._radius}\n')
            if self._original is not None:
                file.write('# params_at_start=' + ' '.join(
                    f'{k}={v}' for k, v in self._original.items()) + '\n')
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        self._log(f'raw data written to {path}')

    def _log(self, text):
        self._node.get_logger().info(text)


def _fmt(value, digits=3):
    return 'n/a' if value is None else f'{value:.{digits}f}'


def _fmt_fit(fit):
    if fit is None:
        return 'not enough moving points'
    return (f'min_duty {fit.min_duty:.4f}, slope {fit.slope:.4f} '
            f'duty/(rad/s) from {fit.n_points} points, rms error '
            f'{fit.rms_error:.4f}')


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--mode', choices=('ff', 'pid', 'all'), default='ff')
    parser.add_argument('--topic', default='/cmd_vel_keyboard',
                        help='TwistStamped input of twist_mux')
    parser.add_argument('--param-node', default=DEFAULT_PARAM_NODE,
                        help='node owning pid_kp / ff_* (found by itself '
                        'if this one does not have them)')
    parser.add_argument('--wheel-radius', type=float,
                        help=f'm, default: {CONTROLLER_NODE} wheel_radius '
                        f'or {DEFAULT_WHEEL_RADIUS}')
    parser.add_argument('--left-joint', default='left_wheel_joint')
    parser.add_argument('--right-joint', default='right_wheel_joint')
    parser.add_argument('--battery-voltage', type=float,
                        help='V, written to the CSV, the slope scales with it')
    parser.add_argument('--note', default='',
                        help='free text for the CSV, e.g. bench / floor')
    parser.add_argument('--csv', help='raw data file, default '
                        'wheel_tuning_<time>.csv in the current directory')
    parser.add_argument('--apply', action='store_true',
                        help='keep the fitted feedforward and the given '
                        'kp/ki/integral_limit instead of restoring')
    parser.add_argument('--handshake-wait', type=float, default=2.0,
                        help='s to wait after a parameter change')

    ff = parser.add_argument_group('feedforward (--mode ff / all)')
    ff.add_argument('--baseline', action='store_true',
                    help='start from min_duty = 0, slope = 0.1 (duty = '
                    'speed / 10), otherwise the current feedforward is used')
    ff.add_argument('--speeds', type=float, nargs='+',
                    help='wheel rad/s in sweep order, > 0 (default with '
                    '--baseline 1.5 .. 5 in 0.25 steps, i.e. duty 0.15 .. '
                    '0.5, else 1 .. 10)')
    ff.add_argument('--direction', choices=('forward', 'reverse', 'both'),
                    default='forward')
    ff.add_argument('--settle', type=float, default=1.0,
                    help='s to wait at each speed before measuring')
    ff.add_argument('--first-settle', type=float, default=2.0,
                    help='extra s for the first point after a handshake')
    ff.add_argument('--measure', type=float, default=3.0,
                    help='s over which the speed is measured')
    ff.add_argument('--stall-threshold', type=float, default=0.2,
                    help='rad/s below which a wheel counts as stalled')
    ff.add_argument('--saturation', type=float, default=0.9,
                    help='points at this duty or more are not fitted')
    ff.add_argument('--fit-min', type=float, default=1.0,
                    help='rad/s, slower points are not fitted')
    ff.add_argument('--fit-max', type=float,
                    help='rad/s, faster points are not fitted (default: '
                    'joint velocity limit + --fit-margin)')
    ff.add_argument('--fit-margin', type=float, default=1.0,
                    help='rad/s above the joint velocity limit still fitted')
    ff.add_argument('--stall-margin', type=float, default=0.025,
                    help='duty, points less than this above the highest '
                    'stalled duty are not fitted')
    ff.add_argument('--outlier', type=float, default=0.4,
                    help='relative speed deviation from the neighbouring '
                    'points above which a point is not fitted')
    ff.add_argument('--iterate', type=int, default=0,
                    help='rounds of apply fit -> measure error -> refit '
                    '(0: only the sweep)')
    ff.add_argument('--tolerance', type=float, default=0.05,
                    help='relative speed error at which --iterate stops')
    ff.add_argument('--verify-speeds', type=float, nargs='+',
                    help='wheel rad/s measured in each --iterate round '
                    '(default 2 .. 10 within the fit range)')

    pid = parser.add_argument_group('PI (--mode pid / all)')
    pid.add_argument('--kp', type=float, nargs='+',
                     help='one value, or a list for a grid search')
    pid.add_argument('--ki', type=float, nargs='+',
                     help='one value, or a list for a grid search')
    pid.add_argument('--integral-limit', type=float)
    pid.add_argument('--steps', type=float, nargs='+', default=[3.0, 6.0, 3.0],
                     help='wheel rad/s targets, from rest, each held '
                     '--step-duration s (default 3 6 3)')
    pid.add_argument('--step-duration', type=float, default=3.0)
    pid.add_argument('--smooth', type=int, default=5,
                     help='samples of the centered moving average')
    pid.add_argument('--settle-band', type=float, default=0.05,
                     help='settling band, fraction of the step')
    pid.add_argument('--settle-band-min', type=float, default=0.3,
                     help='rad/s, lower limit of the settling band, also '
                     'the noise floor of the oscillation check')
    pid.add_argument('--overshoot-weight', type=float, default=1.0,
                     help='weight of the overshoot (fraction of the step) '
                     'in the grid score')

    args = parser.parse_args(argv)
    if args.speeds is None:
        args.speeds = BASELINE_SPEEDS if args.baseline else DEFAULT_SPEEDS
    for speed in args.speeds + (args.verify_speeds or []):
        if not 0.0 < speed <= JOINT_VELOCITY_LIMIT:
            parser.error('--speeds / --verify-speeds must be in '
                         f'(0, {JOINT_VELOCITY_LIMIT}], '
                         f'got {speed}')
    for step in args.steps:
        if abs(step) > JOINT_VELOCITY_LIMIT:
            parser.error(f'--steps must be within +-{JOINT_VELOCITY_LIMIT}, '
                         f'got {step}')
    if args.smooth < 1:
        parser.error('--smooth must be >= 1')
    if args.iterate < 0:
        parser.error('--iterate must be >= 0')
    if not args.tolerance > 0.0:
        parser.error('--tolerance must be > 0')
    if args.csv is None:
        args.csv = datetime.datetime.now().strftime(
            'wheel_tuning_%Y%m%d_%H%M%S.csv')
    return args


def main(argv=None):
    """Run the selected tuning modes, return the exit code."""
    args = parse_args(sys.argv[1:] if argv is None else argv)
    # the reports are printed, keep them in order with the log lines
    sys.stdout.reconfigure(line_buffering=True)
    # without rclpy's handler Ctrl-C is a KeyboardInterrupt and the context
    # stays up, so the robot can still be stopped and the params restored
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = WheelTuning(args)
    run = TuningRun(node, args)
    code = 0
    try:
        if args.battery_voltage is None:
            node.get_logger().warning('no --battery-voltage given, note it: '
                                      'the slope changes with the voltage')
        run.setup()
        if args.mode in ('ff', 'all'):
            run.run_ff()
        if args.mode in ('pid', 'all'):
            run.run_pid(use_fit=args.mode == 'all')
    except KeyboardInterrupt:
        node.get_logger().warning('interrupted, stopping')
        code = 130
    except TuningError as error:
        node.get_logger().error(str(error))
        code = 1
    finally:
        try:
            run.finish()
        except (KeyboardInterrupt, TuningError) as error:
            node.get_logger().error(f'could not restore the parameters: '
                                    f'{error!r}')
            code = code or 1
        run.write_csv(args.csv)
        node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
