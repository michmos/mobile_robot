#include "mobile_robot_controller/speed_limiter.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

using mobile_robot_controller::limitVelocity;

namespace {

constexpr double k_tolerance = 1e-12;
constexpr double k_nan = std::numeric_limits<double>::quiet_NaN();

// values of mobile_robot_nodes/config/controller_manager.yaml (linear.x) and
// its 50 Hz update rate
constexpr double k_accel = 0.3;
constexpr double k_decel = 0.6;
constexpr double k_dt_s = 0.02;
constexpr double k_accelStep = k_accel * k_dt_s; // 0.006
constexpr double k_decelStep = k_decel * k_dt_s; // 0.012

// runs the limiter `steps` times from `start` towards `target`
// @return: the output of every step
std::vector<double> ramp(double start, double target, int steps,
                         double accel = k_accel, double decel = k_decel,
                         double dt_s = k_dt_s) {
  std::vector<double> outputs;
  double v = start;
  for (int i = 0; i < steps; ++i) {
    v = limitVelocity(v, target, accel, decel, dt_s);
    outputs.push_back(v);
  }
  return outputs;
}

} // namespace

// ---------------------------------------------------------------------------
// acceleration
// ---------------------------------------------------------------------------

// the case from #83: 0 -> 0.15 m/s at 0.3 m/s^2 takes 0.5 s = 25 cycles
TEST(SpeedLimiter, StepFromRestRampsInEqualSteps) {
  const auto outputs = ramp(0.0, 0.15, 30);

  for (int i = 0; i < 24; ++i) {
    EXPECT_NEAR(outputs[i], (i + 1) * k_accelStep, k_tolerance)
        << "step " << i + 1;
    EXPECT_LT(outputs[i], 0.15) << "step " << i + 1;
  }
  EXPECT_DOUBLE_EQ(outputs[24], 0.15);
  // holds the target without overshooting
  for (int i = 24; i < 30; ++i) {
    EXPECT_DOUBLE_EQ(outputs[i], 0.15) << "step " << i + 1;
  }
}

TEST(SpeedLimiter, NeverOvershootsWhenTargetIsNoMultipleOfStep) {
  // 0.1 / 0.006 = 16.67 steps: the 17th step is shortened to land on 0.1
  const auto outputs = ramp(0.0, 0.1, 20);

  EXPECT_NEAR(outputs[15], 16 * k_accelStep, k_tolerance);
  for (int i = 16; i < 20; ++i) {
    EXPECT_DOUBLE_EQ(outputs[i], 0.1) << "step " << i + 1;
  }
}

TEST(SpeedLimiter, StepSmallerThanLimitPassesThrough) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.1, 0.104, k_accel, k_decel, k_dt_s), 0.104);
  EXPECT_DOUBLE_EQ(limitVelocity(0.1, 0.095, k_accel, k_decel, k_dt_s), 0.095);
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, -0.005, k_accel, k_decel, k_dt_s),
                   -0.005);
}

TEST(SpeedLimiter, StepExactlyAtLimitPassesThrough) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, k_accelStep, k_accel, k_decel, k_dt_s),
                   k_accelStep);
}

TEST(SpeedLimiter, UnchangedTargetKeepsVelocity) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.15, 0.15, k_accel, k_decel, k_dt_s), 0.15);
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, 0.0, k_accel, k_decel, k_dt_s), 0.0);
}

// ---------------------------------------------------------------------------
// deceleration
// ---------------------------------------------------------------------------

TEST(SpeedLimiter, BrakingUsesDecelerationLimit) {
  // 0.15 -> 0 at 0.6 m/s^2: 12 steps of 0.012, the 13th lands on 0
  const auto outputs = ramp(0.15, 0.0, 15);

  for (int i = 0; i < 12; ++i) {
    EXPECT_NEAR(outputs[i], 0.15 - (i + 1) * k_decelStep, k_tolerance)
        << "step " << i + 1;
    EXPECT_GT(outputs[i], 0.0) << "step " << i + 1;
  }
  for (int i = 12; i < 15; ++i) {
    EXPECT_DOUBLE_EQ(outputs[i], 0.0) << "step " << i + 1;
  }
}

TEST(SpeedLimiter, SlowingDownWithoutStoppingUsesDecelerationLimit) {
  EXPECT_NEAR(limitVelocity(0.15, 0.05, k_accel, k_decel, k_dt_s),
              0.15 - k_decelStep, k_tolerance);
}

// acceleration and deceleration are told apart by |v|, not by the sign of the
// change: speeding up backwards is acceleration
TEST(SpeedLimiter, NegativeDirectionIsSymmetric) {
  const auto up = ramp(0.0, 0.15, 30);
  const auto upBackwards = ramp(0.0, -0.15, 30);
  for (int i = 0; i < 30; ++i) {
    EXPECT_NEAR(upBackwards[i], -up[i], k_tolerance) << "step " << i + 1;
  }

  const auto down = ramp(0.15, 0.0, 15);
  const auto downBackwards = ramp(-0.15, 0.0, 15);
  for (int i = 0; i < 15; ++i) {
    EXPECT_NEAR(downBackwards[i], -down[i], k_tolerance) << "step " << i + 1;
  }

  EXPECT_NEAR(limitVelocity(-0.1, -0.2, k_accel, k_decel, k_dt_s),
              -0.1 - k_accelStep, k_tolerance);
  EXPECT_NEAR(limitVelocity(-0.1, -0.05, k_accel, k_decel, k_dt_s),
              -0.1 + k_decelStep, k_tolerance);
}

// ---------------------------------------------------------------------------
// zero crossing
// ---------------------------------------------------------------------------

// +0.1 -> -0.1: brakes with 0.012 per step to exactly 0 (8 full steps, the
// 9th is shortened to end on 0), then speeds up backwards with 0.006 per step
TEST(SpeedLimiter, ZeroCrossingBrakesToZeroThenAccelerates) {
  const auto outputs = ramp(0.1, -0.1, 30);

  for (int i = 0; i < 8; ++i) {
    EXPECT_NEAR(outputs[i], 0.1 - (i + 1) * k_decelStep, k_tolerance)
        << "step " << i + 1;
  }
  EXPECT_DOUBLE_EQ(outputs[8], 0.0);
  for (int i = 9; i < 25; ++i) {
    EXPECT_NEAR(outputs[i], -(i - 8) * k_accelStep, k_tolerance)
        << "step " << i + 1;
  }
  for (int i = 25; i < 30; ++i) {
    EXPECT_DOUBLE_EQ(outputs[i], -0.1) << "step " << i + 1;
  }
}

TEST(SpeedLimiter, ZeroCrossingIsSymmetric) {
  const auto forward = ramp(0.1, -0.1, 30);
  const auto backward = ramp(-0.1, 0.1, 30);
  for (int i = 0; i < 30; ++i) {
    EXPECT_NEAR(backward[i], -forward[i], k_tolerance) << "step " << i + 1;
  }
}

// a step that could brake past 0 ends on 0 instead of using the rest of the
// step to accelerate the other way
TEST(SpeedLimiter, SingleStepNeverCrossesZero) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.005, -0.1, k_accel, k_decel, k_dt_s), 0.0);
  EXPECT_DOUBLE_EQ(limitVelocity(-0.005, 0.1, k_accel, k_decel, k_dt_s), 0.0);
}

TEST(SpeedLimiter, StartingFromZeroInEitherDirectionAccelerates) {
  EXPECT_NEAR(limitVelocity(0.0, 0.2, k_accel, k_decel, k_dt_s), k_accelStep,
              k_tolerance);
  EXPECT_NEAR(limitVelocity(0.0, -0.2, k_accel, k_decel, k_dt_s), -k_accelStep,
              k_tolerance);
}

// ---------------------------------------------------------------------------
// dt
// ---------------------------------------------------------------------------

TEST(SpeedLimiter, StepScalesWithDt) {
  EXPECT_NEAR(limitVelocity(0.0, 0.15, k_accel, k_decel, 0.01), 0.003,
              k_tolerance);
  EXPECT_NEAR(limitVelocity(0.0, 0.15, k_accel, k_decel, 0.04), 0.012,
              k_tolerance);
  EXPECT_NEAR(limitVelocity(0.15, 0.0, k_accel, k_decel, 0.04), 0.126,
              k_tolerance);

  // 0 -> 0.15 takes 0.5 s regardless of the cycle time
  const auto outputs = ramp(0.0, 0.15, 50, k_accel, k_decel, 0.01);
  EXPECT_LT(outputs[48], 0.15);
  EXPECT_DOUBLE_EQ(outputs[49], 0.15);
}

TEST(SpeedLimiter, ZeroDtKeepsVelocity) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, 0.15, k_accel, k_decel, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(limitVelocity(0.1, 0.0, k_accel, k_decel, 0.0), 0.1);
  EXPECT_DOUBLE_EQ(limitVelocity(0.1, -0.1, k_accel, k_decel, 0.0), 0.1);
}

// ---------------------------------------------------------------------------
// NaN = no limit
// ---------------------------------------------------------------------------

TEST(SpeedLimiter, NaNLimitsPassTargetThrough) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, 0.15, k_nan, k_nan, k_dt_s), 0.15);
  EXPECT_DOUBLE_EQ(limitVelocity(0.15, 0.0, k_nan, k_nan, k_dt_s), 0.0);
  EXPECT_DOUBLE_EQ(limitVelocity(0.1, -0.1, k_nan, k_nan, k_dt_s), -0.1);
  EXPECT_DOUBLE_EQ(limitVelocity(-0.3, 0.2, k_nan, k_nan, k_dt_s), 0.2);
}

// controllers without limits run update() with any period, a period of 0
// must not hold the command back
TEST(SpeedLimiter, NaNLimitsPassThroughEvenWithZeroDt) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, 0.15, k_nan, k_nan, 0.0), 0.15);
  EXPECT_DOUBLE_EQ(limitVelocity(0.15, 0.0, k_nan, k_nan, 0.0), 0.0);
}

TEST(SpeedLimiter, NaNAccelerationOnlyUnlimitsSpeedingUp) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.0, 0.15, k_nan, k_decel, k_dt_s), 0.15);
  EXPECT_NEAR(limitVelocity(0.15, 0.0, k_nan, k_decel, k_dt_s),
              0.15 - k_decelStep, k_tolerance);
}

TEST(SpeedLimiter, NaNDecelerationOnlyUnlimitsSlowingDown) {
  EXPECT_DOUBLE_EQ(limitVelocity(0.15, 0.0, k_accel, k_nan, k_dt_s), 0.0);
  EXPECT_DOUBLE_EQ(limitVelocity(-0.15, -0.05, k_accel, k_nan, k_dt_s), -0.05);
  EXPECT_NEAR(limitVelocity(0.0, 0.15, k_accel, k_nan, k_dt_s), k_accelStep,
              k_tolerance);
}
