#include "mobile_robot_controller/speed_limiter.hpp"

#include <cmath>

namespace mobile_robot_controller {

double limitVelocity(double previous, double target, double maxAcceleration,
                     double maxDeceleration, double dt_s) {
  if (std::isnan(maxAcceleration) && std::isnan(maxDeceleration)) {
    return target;
  }

  // opposite signs: brake towards 0 first, the other direction starts in a
  // later step
  const bool crossesZero = previous * target < 0.0;
  const double goal = crossesZero ? 0.0 : target;
  const double limit =
      std::abs(goal) > std::abs(previous) ? maxAcceleration : maxDeceleration;
  if (std::isnan(limit)) {
    return goal;
  }
  if (dt_s <= 0.0) {
    return previous;
  }

  const double maxStep = limit * dt_s;
  const double diff = goal - previous;
  if (std::abs(diff) <= maxStep) {
    return goal;
  }
  return previous + std::copysign(maxStep, diff);
}

} // namespace mobile_robot_controller
