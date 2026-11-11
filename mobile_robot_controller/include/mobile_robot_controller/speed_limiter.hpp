#ifndef MOBILE_ROBOT_CONTROLLER_SPEED_LIMITER_HPP
#define MOBILE_ROBOT_CONTROLLER_SPEED_LIMITER_HPP

// acceleration limiting of commanded velocities, kept free of ROS state so it
// can be unit tested

namespace mobile_robot_controller {

// moves `previous` (last limited command) towards `target` by at most
// limit * dt, with limit = maxAcceleration while |v| grows and
// maxDeceleration while |v| shrinks
// - reaches `target` exactly once it is within one step
// - a sign change of `target` relative to `previous` brakes towards 0 first:
//   a step never crosses 0, it ends at exactly 0 instead
// - a NaN limit means no limit in that direction; with both limits NaN the
//   result is `target`
// - dt <= 0 leaves `previous` unchanged in limited directions
// limits in m/s^2 or rad/s^2, dt in s
// @return: the limited velocity
double limitVelocity(double previous, double target, double maxAcceleration,
                     double maxDeceleration, double dt);

} // namespace mobile_robot_controller

#endif
