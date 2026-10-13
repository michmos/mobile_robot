#include "mobile_robot_controller/kinematics.hpp"

#include "tf2/LinearMath/Quaternion.hpp"

#include <cmath>

namespace mobile_robot_controller {

std::pair<double, double> inverseKinematics(double linearVel,
                                            double angularVel,
                                            double wheelSeparation,
                                            double wheelRadius) {
  // velocity difference rule for rigid bodies
  double leftWheelSpeed = linearVel - angularVel * wheelSeparation / 2.0;
  double rightWheelSpeed = linearVel + angularVel * wheelSeparation / 2.0;

  // convert wheel linear speed (m/s) to wheel angular velocity (rad/s)
  double leftWheelVel = leftWheelSpeed / wheelRadius;
  double rightWheelVel = rightWheelSpeed / wheelRadius;

  return {leftWheelVel, rightWheelVel};
}

BodyDelta integrateOdometry(Pose &pose, double deltaLeft, double deltaRight,
                            double wheelSeparation, double wheelRadius) {
  double distanceLeft = deltaLeft * wheelRadius;
  double distanceRight = deltaRight * wheelRadius;
  double distanceCenter = (distanceLeft + distanceRight) / 2.0;
  double deltaHeading = (distanceRight - distanceLeft) / wheelSeparation;

  // mid point approx to get new pose
  double headingMid = pose.heading + deltaHeading / 2.0;
  pose.x += distanceCenter * cos(headingMid);
  pose.y += distanceCenter * sin(headingMid);
  pose.heading += deltaHeading;

  return {distanceCenter, deltaHeading};
}

void growPoseVariance(PoseVariance &variance, const BodyDelta &delta,
                      double positionVariancePerMeter,
                      double yawVariancePerRadian) {
  variance.position += positionVariancePerMeter * std::fabs(delta.distance);
  variance.yaw += yawVariancePerRadian * std::fabs(delta.heading);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion msg;
  msg.x = 0.0;
  msg.y = 0.0;
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

void setDiagonalCovariance(std::array<double, 36> &covariance,
                           double xVariance, double yVariance,
                           double zVariance, double rollVariance,
                           double pitchVariance, double yawVariance) {
  // ! one dimenstional array !
  covariance[0] = xVariance;      // (0, 0)
  covariance[7] = yVariance;      // (1, 1)
  covariance[14] = zVariance;     // (2, 2)
  covariance[21] = rollVariance;  // (3, 3)
  covariance[28] = pitchVariance; // (4, 4)
  covariance[35] = yawVariance;   // (5, 5)
}

} // namespace mobile_robot_controller
