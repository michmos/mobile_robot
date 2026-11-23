#ifndef MOBILE_ROBOT_CONTROLLER_SPEED_LIMITER_HPP
#define MOBILE_ROBOT_CONTROLLER_SPEED_LIMITER_HPP

// velocity and acceleration limiting of commanded velocities, kept free of ROS
// state so it can be unit tested

namespace mobile_robot_controller {

// moves `previous` (last limited command) towards `target` by at most
// limit * dt_s, with limit = maxAcceleration while |v| grows and
// maxDeceleration while |v| shrinks
// - reaches `target` exactly once it is within one step
// - a sign change of `target` relative to `previous` brakes towards 0 first:
//   a step never crosses 0, it ends at exactly 0 instead
// - a NaN limit means no limit in that direction; with both limits NaN the
//   result is `target`
// - dt_s <= 0 leaves `previous` unchanged in limited directions
// limits in m/s^2 or rad/s^2, dt_s in s
// @return: the limited velocity
double limitAcceleration(double previous, double target, double maxAcceleration,
                         double maxDeceleration, double dt_s);

// clamps `v` to [-maxVelocity, maxVelocity], symmetric for both directions
// - a NaN limit means no limit, the result is `v`
// - applied to the target before limitAcceleration(), so a ramp ends at
//   the limit
// maxVelocity >= 0 in m/s or rad/s
// @return: the clamped velocity
double clampVelocity(double v, double maxVelocity);

} // namespace mobile_robot_controller

#endif
