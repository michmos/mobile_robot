#include "mobile_robot_controller/speed_limiter.hpp"

namespace mobile_robot_controller {

// TODO(#83): stub, does not limit yet
double limitVelocity(double, double target, double, double, double) {
  return target;
}

} // namespace mobile_robot_controller
