#ifndef MOBILE_ROBOT_CONTROLLER_KINEMATICS_HPP
#define MOBILE_ROBOT_CONTROLLER_KINEMATICS_HPP

// diff-drive kinematics and odometry math of MobileRobotController, kept free
// of ROS state so it can be unit tested

#include <geometry_msgs/msg/quaternion.hpp>

#include <array>
#include <utility>

namespace mobile_robot_controller {

// dead-reckoned planar pose, heading in rad (accumulated, not wrapped)
struct Pose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
};

// accumulated pose variance - x and y share one value
struct PoseVariance {
  double position = 0.0;
  double yaw = 0.0;
};

// body motion of one odometry step: distance driven by the base center (m)
// and heading change (rad)
struct BodyDelta {
  double distance = 0.0;
  double heading = 0.0;
};

// inverse kinematics: body twist (m/s, rad/s) -> {leftWheelVel,
// rightWheelVel} (rad/s)
std::pair<double, double> inverseKinematics(double linearVel, double angularVel,
                                            double wheelSeparation,
                                            double wheelRadius);

// forward kinematics + midpoint integration: moves `pose` by the wheel
// rotation deltas (rad) since the last step
// @return: the body motion of this step
BodyDelta integrateOdometry(Pose &pose, double deltaLeft, double deltaRight,
                            double wheelSeparation, double wheelRadius);

// grows `variance` by how far the robot actually moved in `delta`
void growPoseVariance(PoseVariance &variance, const BodyDelta &delta,
                      double positionVariancePerMeter,
                      double yawVariancePerRadian);

// orientation of a planar heading (rotation about z only)
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

// fills the diagonal of a 6x6 row-major covariance array (ordered
// [x, y, z, roll, pitch, yaw], or [vx, vy, vz, wx, wy, wz] for twist)
// from per-axis variances, leaving all covariances (off-diagonal entries)
// at their default of zero
void setDiagonalCovariance(std::array<double, 36> &covariance, double xVariance,
                           double yVariance, double zVariance,
                           double rollVariance, double pitchVariance,
                           double yawVariance);

} // namespace mobile_robot_controller

#endif
