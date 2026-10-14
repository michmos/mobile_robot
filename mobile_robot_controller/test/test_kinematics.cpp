#include "mobile_robot_controller/kinematics.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

using mobile_robot_controller::BodyDelta;
using mobile_robot_controller::growPoseVariance;
using mobile_robot_controller::integrateOdometry;
using mobile_robot_controller::inverseKinematics;
using mobile_robot_controller::Pose;
using mobile_robot_controller::PoseVariance;
using mobile_robot_controller::setDiagonalCovariance;
using mobile_robot_controller::yawToQuaternion;

namespace {

constexpr double k_pi = M_PI;
constexpr double k_eps = 1e-12;

// real robot, see mobile_robot_nodes/description/properties.xacro
constexpr double k_robotSeparation = 0.191;
constexpr double k_robotRadius = 0.0335;

// round numbers for hand-checked cases
constexpr double k_separation = 0.2;
constexpr double k_radius = 0.05;

// real robot, see mobile_robot_nodes/config/controller_manager.yaml
constexpr double k_positionVariancePerMeter = 0.0025;
constexpr double k_yawVariancePerRadian = 0.01;

// wheel rotation (rad) that moves the base center along an arc of `radius`
// (m, positive = turning left) by `angle` (rad)
std::pair<double, double> arcWheelDeltas(double radius, double angle,
                                         double separation,
                                         double wheelRadius) {
  return {(radius - separation / 2.0) * angle / wheelRadius,
          (radius + separation / 2.0) * angle / wheelRadius};
}

} // namespace

// ---------------------------------------------------------------------------
// inverse kinematics
// ---------------------------------------------------------------------------

TEST(InverseKinematics, PureForwardTurnsBothWheelsEqually) {
  const auto [left, right] =
      inverseKinematics(0.2, 0.0, k_separation, k_radius);
  EXPECT_NEAR(left, 4.0, k_eps);
  EXPECT_NEAR(right, 4.0, k_eps);
}

TEST(InverseKinematics, PureRotationTurnsWheelsOppositeAndEqual) {
  // turning left (positive z): right wheel forward, left wheel backward
  const auto [left, right] =
      inverseKinematics(0.0, 1.0, k_separation, k_radius);
  EXPECT_NEAR(left, -2.0, k_eps);
  EXPECT_NEAR(right, 2.0, k_eps);
}

TEST(InverseKinematics, CombinedAddsRotationOnTopOfForward) {
  const auto [left, right] =
      inverseKinematics(0.2, 1.0, k_separation, k_radius);
  EXPECT_NEAR(left, 2.0, k_eps);
  EXPECT_NEAR(right, 6.0, k_eps);
}

TEST(InverseKinematics, ZeroTwistGivesZeroWheelVelocity) {
  const auto [left, right] =
      inverseKinematics(0.0, 0.0, k_separation, k_radius);
  EXPECT_NEAR(left, 0.0, k_eps);
  EXPECT_NEAR(right, 0.0, k_eps);
}

TEST(InverseKinematics, NegativeTwistMirrorsPositiveTwist) {
  // reversing while turning right must be the exact negation of driving
  // forward while turning left
  const auto [left, right] =
      inverseKinematics(-0.2, -1.0, k_separation, k_radius);
  EXPECT_NEAR(left, -2.0, k_eps);
  EXPECT_NEAR(right, -6.0, k_eps);
}

TEST(InverseKinematics, RealRobotValues) {
  // 0.1 m/s forward, 0.5 rad/s left on the real robot
  const auto [left, right] =
      inverseKinematics(0.1, 0.5, k_robotSeparation, k_robotRadius);
  EXPECT_NEAR(left, (0.1 - 0.5 * 0.0955) / 0.0335, 1e-9);
  EXPECT_NEAR(right, (0.1 + 0.5 * 0.0955) / 0.0335, 1e-9);
}

// wheel speeds commanded for a twist must be read back as the same twist by
// the odometry (forward kinematics), otherwise the robot drifts from cmd_vel
class KinematicsRoundTrip
    : public ::testing::TestWithParam<std::pair<double, double>> {};

TEST_P(KinematicsRoundTrip, WheelSpeedsMapBackToTwist) {
  const auto [linear, angular] = GetParam();
  constexpr double dt = 0.01;

  const auto [leftVel, rightVel] =
      inverseKinematics(linear, angular, k_robotSeparation, k_robotRadius);
  Pose pose;
  const BodyDelta delta = integrateOdometry(pose, leftVel * dt, rightVel * dt,
                                            k_robotSeparation, k_robotRadius);

  EXPECT_NEAR(delta.distance / dt, linear, 1e-9);
  EXPECT_NEAR(delta.heading / dt, angular, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
    Twists, KinematicsRoundTrip,
    ::testing::Values(std::make_pair(0.0, 0.0), std::make_pair(0.3, 0.0),
                      std::make_pair(0.0, 2.0), std::make_pair(0.2, 1.0),
                      std::make_pair(-0.2, 1.0), std::make_pair(0.2, -1.0),
                      std::make_pair(-0.15, -0.7)));

// ---------------------------------------------------------------------------
// odometry
// ---------------------------------------------------------------------------

TEST(Odometry, StraightLineMovesAlongHeading) {
  Pose pose;
  const BodyDelta delta =
      integrateOdometry(pose, 2.0, 2.0, k_separation, k_radius);

  EXPECT_NEAR(delta.distance, 0.1, k_eps);
  EXPECT_NEAR(delta.heading, 0.0, k_eps);
  EXPECT_NEAR(pose.x, 0.1, k_eps);
  EXPECT_NEAR(pose.y, 0.0, k_eps);
  EXPECT_NEAR(pose.heading, 0.0, k_eps);
}

TEST(Odometry, StraightLineFollowsCurrentHeading) {
  // facing +y, a straight drive must only change y
  Pose pose{1.0, 2.0, k_pi / 2.0};
  integrateOdometry(pose, 2.0, 2.0, k_separation, k_radius);

  EXPECT_NEAR(pose.x, 1.0, k_eps);
  EXPECT_NEAR(pose.y, 2.1, k_eps);
  EXPECT_NEAR(pose.heading, k_pi / 2.0, k_eps);
}

TEST(Odometry, DrivingBackwardsMovesAgainstHeading) {
  Pose pose;
  const BodyDelta delta =
      integrateOdometry(pose, -2.0, -2.0, k_separation, k_radius);

  EXPECT_NEAR(delta.distance, -0.1, k_eps);
  EXPECT_NEAR(pose.x, -0.1, k_eps);
  EXPECT_NEAR(pose.y, 0.0, k_eps);
  EXPECT_NEAR(pose.heading, 0.0, k_eps);
}

TEST(Odometry, InPlaceRotationDoesNotTranslate) {
  Pose pose{0.5, -0.5, 0.3};
  const BodyDelta delta =
      integrateOdometry(pose, -1.0, 1.0, k_separation, k_radius);

  // each wheel travels 0.05 m on a circle of radius separation / 2
  EXPECT_NEAR(delta.distance, 0.0, k_eps);
  EXPECT_NEAR(delta.heading, 0.5, k_eps);
  EXPECT_NEAR(pose.x, 0.5, k_eps);
  EXPECT_NEAR(pose.y, -0.5, k_eps);
  EXPECT_NEAR(pose.heading, 0.8, k_eps);
}

TEST(Odometry, ZeroMotionKeepsPose) {
  Pose pose{0.5, -0.5, 0.3};
  const BodyDelta delta =
      integrateOdometry(pose, 0.0, 0.0, k_separation, k_radius);

  EXPECT_NEAR(delta.distance, 0.0, k_eps);
  EXPECT_NEAR(delta.heading, 0.0, k_eps);
  EXPECT_NEAR(pose.x, 0.5, k_eps);
  EXPECT_NEAR(pose.y, -0.5, k_eps);
  EXPECT_NEAR(pose.heading, 0.3, k_eps);
}

TEST(Odometry, QuarterCircleMatchesClosedFormArc) {
  // real robot driving a left quarter circle of radius 0.5 m in 100 control
  // cycles; closed form end pose is (R, R, pi/2)
  constexpr double arcRadius = 0.5;
  constexpr int steps = 100;
  const auto [dLeft, dRight] = arcWheelDeltas(arcRadius, (k_pi / 2.0) / steps,
                                              k_robotSeparation, k_robotRadius);

  Pose pose;
  for (int i = 0; i < steps; ++i) {
    integrateOdometry(pose, dLeft, dRight, k_robotSeparation, k_robotRadius);
  }

  // midpoint integration overestimates each chord by ~dTheta^2 / 24
  // (~1e-5 relative here), well below a millimeter
  EXPECT_NEAR(pose.x, arcRadius, 1e-4);
  EXPECT_NEAR(pose.y, arcRadius, 1e-4);
  EXPECT_NEAR(pose.heading, k_pi / 2.0, 1e-9);
}

TEST(Odometry, HeadingCrossesPiContinuously) {
  // heading is accumulated, not wrapped: crossing +pi must neither jump nor
  // disturb the position update, the quaternion takes care of the
  // periodicity (see Quaternion.HeadingPastPiIsSameOrientationAsWrapped)
  Pose pose{0.0, 0.0, k_pi - 0.05};
  integrateOdometry(pose, -1.0, 1.0, k_separation, k_radius); // +0.5 rad

  EXPECT_NEAR(pose.heading, k_pi + 0.45, k_eps);

  integrateOdometry(pose, 2.0, 2.0, k_separation, k_radius); // 0.1 m
  EXPECT_NEAR(pose.x, 0.1 * std::cos(k_pi + 0.45), k_eps);
  EXPECT_NEAR(pose.y, 0.1 * std::sin(k_pi + 0.45), k_eps);
}

TEST(Odometry, HeadingCrossesMinusPiContinuously) {
  Pose pose{0.0, 0.0, -k_pi + 0.05};
  integrateOdometry(pose, 1.0, -1.0, k_separation, k_radius); // -0.5 rad

  EXPECT_NEAR(pose.heading, -k_pi - 0.45, k_eps);

  integrateOdometry(pose, 2.0, 2.0, k_separation, k_radius); // 0.1 m
  EXPECT_NEAR(pose.x, 0.1 * std::cos(-k_pi - 0.45), k_eps);
  EXPECT_NEAR(pose.y, 0.1 * std::sin(-k_pi - 0.45), k_eps);
}

TEST(Odometry, ManySmallStepsMatchOneBigStep) {
  // a late control cycle delivers the wheel delta of many cycles at once;
  // for constant curvature the midpoint heading is exactly the chord
  // direction, only the chord length is off by ~dTheta^2 / 24
  constexpr double arcRadius = 1.0;
  constexpr double angle = 0.2;
  constexpr int steps = 100;
  const auto [bigLeft, bigRight] =
      arcWheelDeltas(arcRadius, angle, k_robotSeparation, k_robotRadius);

  Pose small;
  for (int i = 0; i < steps; ++i) {
    integrateOdometry(small, bigLeft / steps, bigRight / steps,
                      k_robotSeparation, k_robotRadius);
  }
  Pose big;
  integrateOdometry(big, bigLeft, bigRight, k_robotSeparation, k_robotRadius);

  EXPECT_NEAR(big.x, small.x, 1e-3);
  EXPECT_NEAR(big.y, small.y, 1e-3);
  EXPECT_NEAR(big.heading, small.heading, 1e-9);
  EXPECT_NEAR(std::atan2(big.y, big.x), std::atan2(small.y, small.x), 1e-9);
}

TEST(Odometry, StraightLineIsIndependentOfStepCount) {
  Pose small;
  for (int i = 0; i < 10; ++i) {
    integrateOdometry(small, 0.2, 0.2, k_separation, k_radius);
  }
  Pose big;
  integrateOdometry(big, 2.0, 2.0, k_separation, k_radius);

  EXPECT_NEAR(big.x, small.x, k_eps);
  EXPECT_NEAR(big.y, small.y, k_eps);
}

// ---------------------------------------------------------------------------
// covariance
// ---------------------------------------------------------------------------

TEST(Covariance, PositionVarianceGrowsWithDistanceDriven) {
  PoseVariance variance;
  growPoseVariance(variance, {0.4, 0.0}, k_positionVariancePerMeter,
                   k_yawVariancePerRadian);
  EXPECT_NEAR(variance.position, 0.001, k_eps);
  EXPECT_NEAR(variance.yaw, 0.0, k_eps);

  growPoseVariance(variance, {0.6, 0.0}, k_positionVariancePerMeter,
                   k_yawVariancePerRadian);
  EXPECT_NEAR(variance.position, 0.0025, k_eps);
}

TEST(Covariance, DrivingBackwardsAlsoGrowsVariance) {
  // uncertainty grows with the distance driven, not the net displacement
  PoseVariance variance;
  growPoseVariance(variance, {0.5, 0.0}, k_positionVariancePerMeter,
                   k_yawVariancePerRadian);
  growPoseVariance(variance, {-0.5, 0.0}, k_positionVariancePerMeter,
                   k_yawVariancePerRadian);
  EXPECT_NEAR(variance.position, 0.0025, k_eps);
}

TEST(Covariance, YawVarianceGrowsWithRotationInEitherDirection) {
  PoseVariance variance;
  growPoseVariance(variance, {0.0, 1.0}, k_positionVariancePerMeter,
                   k_yawVariancePerRadian);
  growPoseVariance(variance, {0.0, -0.5}, k_positionVariancePerMeter,
                   k_yawVariancePerRadian);
  EXPECT_NEAR(variance.yaw, 0.015, k_eps);
  EXPECT_NEAR(variance.position, 0.0, k_eps);
}

TEST(Covariance, StandingStillDoesNotGrowVariance) {
  PoseVariance variance{0.1, 0.2};
  for (int i = 0; i < 1000; ++i) {
    Pose pose;
    const BodyDelta delta =
        integrateOdometry(pose, 0.0, 0.0, k_robotSeparation, k_robotRadius);
    growPoseVariance(variance, delta, k_positionVariancePerMeter,
                     k_yawVariancePerRadian);
  }
  EXPECT_NEAR(variance.position, 0.1, k_eps);
  EXPECT_NEAR(variance.yaw, 0.2, k_eps);
}

TEST(Covariance, AccumulatesOverOdometrySteps) {
  // 1 m straight in 10 cycles on the real robot
  const double wheelDelta = 0.1 / k_robotRadius;
  Pose pose;
  PoseVariance variance;
  for (int i = 0; i < 10; ++i) {
    const BodyDelta delta = integrateOdometry(pose, wheelDelta, wheelDelta,
                                              k_robotSeparation, k_robotRadius);
    growPoseVariance(variance, delta, k_positionVariancePerMeter,
                     k_yawVariancePerRadian);
  }
  EXPECT_NEAR(variance.position, k_positionVariancePerMeter * 1.0, 1e-12);
  EXPECT_NEAR(variance.yaw, 0.0, k_eps);
}

TEST(Covariance, SetDiagonalFillsOnlyTheDiagonal) {
  std::array<double, 36> covariance{};
  setDiagonalCovariance(covariance, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t col = 0; col < 6; ++col) {
      const double expected = row == col ? static_cast<double>(row + 1) : 0.0;
      EXPECT_EQ(covariance[row * 6 + col], expected)
          << "row " << row << ", col " << col;
    }
  }
}

// ---------------------------------------------------------------------------
// quaternion
// ---------------------------------------------------------------------------

class YawToQuaternion : public ::testing::TestWithParam<double> {};

TEST_P(YawToQuaternion, IsNormalizedRotationAboutZ) {
  const double yaw = GetParam();
  const auto q = yawToQuaternion(yaw);

  EXPECT_NEAR(q.x, 0.0, k_eps);
  EXPECT_NEAR(q.y, 0.0, k_eps);
  EXPECT_NEAR(q.z, std::sin(yaw / 2.0), k_eps);
  EXPECT_NEAR(q.w, std::cos(yaw / 2.0), k_eps);
  EXPECT_NEAR(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w, 1.0, k_eps);
}

INSTANTIATE_TEST_SUITE_P(Yaws, YawToQuaternion,
                         ::testing::Values(0.0, k_pi / 2.0, k_pi, -k_pi / 2.0,
                                           -k_pi, 0.3, 3.0 * k_pi / 2.0));

TEST(Quaternion, KnownValues) {
  const double halfSqrt2 = std::sqrt(2.0) / 2.0;

  auto q = yawToQuaternion(0.0);
  EXPECT_NEAR(q.z, 0.0, k_eps);
  EXPECT_NEAR(q.w, 1.0, k_eps);

  q = yawToQuaternion(k_pi / 2.0);
  EXPECT_NEAR(q.z, halfSqrt2, k_eps);
  EXPECT_NEAR(q.w, halfSqrt2, k_eps);

  q = yawToQuaternion(k_pi);
  EXPECT_NEAR(q.z, 1.0, k_eps);
  EXPECT_NEAR(q.w, 0.0, k_eps);

  q = yawToQuaternion(-k_pi / 2.0);
  EXPECT_NEAR(q.z, -halfSqrt2, k_eps);
  EXPECT_NEAR(q.w, halfSqrt2, k_eps);
}

TEST(Quaternion, HeadingPastPiIsSameOrientationAsWrapped) {
  // the controller never wraps the accumulated heading, so pi + 0.45 must
  // publish the same orientation as -pi + 0.45 (q and -q are the same
  // rotation)
  const auto unwrapped = yawToQuaternion(k_pi + 0.45);
  const auto wrapped = yawToQuaternion(-k_pi + 0.45);

  EXPECT_NEAR(std::fabs(unwrapped.z * wrapped.z + unwrapped.w * wrapped.w), 1.0,
              k_eps);
}
