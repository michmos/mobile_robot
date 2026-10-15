// host tests for the firmware's feedforward + PI motor controller,
// compiled against test/arduino_stub/Arduino.h
#include "inc/MotorControl.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

// duties are sums of a few floats, far below this
constexpr float k_tol = 1e-5f;

// gains in the order of configure()
struct Gains {
  float min_duty_left = 0.0f;
  float min_duty_right = 0.0f;
  float slope_left = 0.1f;
  float slope_right = 0.1f;
  float kp = 0.0f;
  float ki = 0.0f;
  float integral_limit = 0.0f;
};

class MotorControlTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }

  void configure(const Gains &g) {
    mc_.configure(g.min_duty_left, g.min_duty_right, g.slope_left,
                  g.slope_right, g.kp, g.ki, g.integral_limit);
  }

  MotorControl mc_;
};

///////////////////////////////////////////////////////////////////////////////
// feedforward
///////////////////////////////////////////////////////////////////////////////

TEST_F(MotorControlTest, FeedforwardZeroCommandGivesZero) {
  Gains g;
  g.min_duty_left = 0.2f;
  g.min_duty_right = 0.3f;
  configure(g);

  // the min-duty offset must not kick in at standstill, or a stopped robot
  // would creep
  EXPECT_NEAR(mc_.feedforwardLeft(0.0f), 0.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardRight(0.0f), 0.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(-0.0f), 0.0f, k_tol);
}

TEST_F(MotorControlTest, FeedforwardSignFollowsCommand) {
  Gains g;
  g.min_duty_left = 0.2f;
  configure(g);

  // 0.2 + 0.1 * 2
  EXPECT_NEAR(mc_.feedforwardLeft(2.0f), 0.4f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(-2.0f), -0.4f, k_tol);
}

TEST_F(MotorControlTest, FeedforwardAddsMinDutyOffset) {
  Gains g;
  g.min_duty_left = 0.2f;
  configure(g);

  // even the smallest command gets the duty that overcomes static friction
  EXPECT_NEAR(mc_.feedforwardLeft(1e-3f), 0.2f + 1e-4f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(-1e-3f), -0.2f - 1e-4f, k_tol);
  // and above it the duty grows with the slope
  EXPECT_NEAR(mc_.feedforwardLeft(4.0f) - mc_.feedforwardLeft(2.0f), 0.2f,
              k_tol);
}

TEST_F(MotorControlTest, FeedforwardClampsToDutyRange) {
  Gains g;
  g.min_duty_left = 0.2f;
  configure(g);

  // 0.2 + 0.1 * 8 hits the limit exactly
  EXPECT_NEAR(mc_.feedforwardLeft(8.0f), 1.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(8.1f), 1.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(100.0f), 1.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(-100.0f), -1.0f, k_tol);
}

TEST_F(MotorControlTest, FeedforwardInfiniteCommandStops) {
  Gains g;
  g.min_duty_left = 0.2f;
  configure(g);

  // inf is no speed the robot can reach, it only shows up when something
  // upstream went wrong (#22). Treated like NaN: stop, don't saturate
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_NEAR(mc_.feedforwardLeft(inf), 0.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardLeft(-inf), 0.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardRight(inf), 0.0f, k_tol);
}

TEST_F(MotorControlTest, FeedforwardUsesPerWheelParameters) {
  Gains g;
  g.min_duty_left = 0.1f;
  g.min_duty_right = 0.3f;
  g.slope_left = 0.1f;
  g.slope_right = 0.2f;
  configure(g);

  EXPECT_NEAR(mc_.feedforwardLeft(1.0f), 0.2f, k_tol);
  EXPECT_NEAR(mc_.feedforwardRight(1.0f), 0.5f, k_tol);
}

TEST_F(MotorControlTest, UnconfiguredControllerOutputsZero) {
  // the firmware only drives after a config was applied, but a default
  // constructed controller must not move the motors either
  MotorControl mc;
  EXPECT_NEAR(mc.feedforwardLeft(5.0f), 0.0f, k_tol);
  EXPECT_NEAR(mc.feedforwardRight(-5.0f), 0.0f, k_tol);
  EXPECT_NEAR(mc.computeDutyLeft(5.0f, 0.0f, 0.01f), 0.0f, k_tol);
  EXPECT_NEAR(mc.computeDutyRight(-5.0f, 0.0f, 0.01f), 0.0f, k_tol);
}

TEST_F(MotorControlTest, NanCommandDoesNotDrive) {
  Gains g;
  g.min_duty_left = 0.2f;
  g.kp = 0.5f;
  g.ki = 2.0f;
  g.integral_limit = 0.3f;
  configure(g);

  // strtof() accepts "nan", so "M,nan,0" passes MotorCmd::init(). A command
  // that isn't a number must not turn into full duty
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_NEAR(mc_.feedforwardLeft(nan), 0.0f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(nan, 0.0f, 0.01f), 0.0f, k_tol);
  EXPECT_NEAR(mc_.feedforwardRight(nan), 0.0f, k_tol);
  EXPECT_NEAR(mc_.computeDutyRight(nan, 0.0f, 0.01f), 0.0f, k_tol);
}

///////////////////////////////////////////////////////////////////////////////
// PI
///////////////////////////////////////////////////////////////////////////////

TEST_F(MotorControlTest, ProportionalResponse) {
  Gains g;
  g.min_duty_left = 0.2f;
  g.kp = 0.5f;
  configure(g);

  // ff(2) = 0.4, plus kp * (cmd - curr)
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.9f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 3.0f, 0.01f), -0.1f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 2.0f, 0.01f), 0.4f, k_tol);
}

TEST_F(MotorControlTest, ProportionalResponseBackwards) {
  Gains g;
  g.min_duty_left = 0.2f;
  g.kp = 0.5f;
  configure(g);

  // ff(-2) = -0.4, a wheel too slow backwards gets pushed further back
  EXPECT_NEAR(mc_.computeDutyLeft(-2.0f, -1.0f, 0.01f), -0.9f, k_tol);
}

TEST_F(MotorControlTest, ZeroCommandGivesZeroWhateverTheVelocity) {
  Gains g;
  g.min_duty_left = 0.2f;
  g.kp = 0.5f;
  configure(g);

  // a stop command holds still instead of servoing on velocity noise
  EXPECT_NEAR(mc_.computeDutyLeft(0.0f, 5.0f, 0.01f), 0.0f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(0.0f, -5.0f, 0.01f), 0.0f, k_tol);
}

TEST_F(MotorControlTest, PiOutputIsClamped) {
  Gains g;
  g.kp = 10.0f;
  configure(g);

  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 0.0f, 0.01f), 1.0f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(-2.0f, 0.0f, 0.01f), -1.0f, k_tol);
}

// gains for the integral tests: ff(2) = 0.2, no P, the integral grows by
// ki * err * dt = 0.02 per call with err 1 and dt 0.01
Gains integralGains(float limit = 1.0f) {
  Gains g;
  g.ki = 2.0f;
  g.integral_limit = limit;
  return g;
}

TEST_F(MotorControlTest, IntegralAccumulatesOverCalls) {
  configure(integralGains());

  // the integral is updated after the duty is computed, so it shows up one
  // call later
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.20f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.22f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.24f, k_tol);
}

TEST_F(MotorControlTest, IntegralDecreasesOnNegativeError) {
  configure(integralGains());

  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 3.0f, 0.01f), 0.20f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 3.0f, 0.01f), 0.18f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 3.0f, 0.01f), 0.16f, k_tol);
}

TEST_F(MotorControlTest, IntegralScalesWithDt) {
  configure(integralGains());

  mc_.computeDutyLeft(2.0f, 1.0f, 0.05f);
  // 2 * 1 * 0.05
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.30f, k_tol);
}

TEST_F(MotorControlTest, ZeroDtDoesNotIntegrate) {
  configure(integralGains());

  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.0f), 0.20f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.0f), 0.20f, k_tol);
}

TEST_F(MotorControlTest, ZeroCommandResetsIntegral) {
  configure(integralGains());

  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  EXPECT_NEAR(mc_.computeDutyLeft(0.0f, 1.0f, 0.01f), 0.0f, k_tol);

  // restarting after a stop must not carry the old integral (0.04) along
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.20f, k_tol);
}

TEST_F(MotorControlTest, IntegralsArePerWheel) {
  configure(integralGains());

  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);

  EXPECT_NEAR(mc_.computeDutyRight(2.0f, 1.0f, 0.01f), 0.20f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.24f, k_tol);
}

TEST_F(MotorControlTest, ZeroIntegralLimitDisablesIntegral) {
  configure(integralGains(0.0f));

  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.20f, k_tol);
}

TEST_F(MotorControlTest, NonFiniteCommandStopsAndResetsIntegral) {
  configure(integralGains());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  for (const float cmd : {nan, inf, -inf}) {
    mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
    mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
    EXPECT_NEAR(mc_.computeDutyLeft(cmd, 1.0f, 0.01f), 0.0f, k_tol) << cmd;

    // like a stop command: the integral (0.04) must not survive it, and a
    // NaN must not latch into it at +integral_limit
    EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.20f, k_tol) << cmd;
    mc_.reset();
  }
}

TEST_F(MotorControlTest, NonFiniteVelocityStopsAndResetsIntegral) {
  configure(integralGains());
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  // the firmware divides the tick delta by dt, so dt == 0 gives inf (ticks
  // moved) or NaN (0 / 0). Servoing on that would saturate the wheel
  for (const float vel : {nan, inf, -inf}) {
    mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
    mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
    EXPECT_NEAR(mc_.computeDutyLeft(2.0f, vel, 0.01f), 0.0f, k_tol) << vel;
    EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.20f, k_tol) << vel;
    mc_.reset();
  }
}

// +1 runs the scenario forwards, -1 mirrored backwards
class MotorControlDirectionTest : public MotorControlTest,
                                  public ::testing::WithParamInterface<float> {
};

TEST_P(MotorControlDirectionTest, IntegralClampsAtLimit) {
  const float s = GetParam();
  configure(integralGains(0.05f));

  // the integral goes 0.02, 0.04, then stops at the limit of 0.05
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.01f), s * 0.20f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.01f), s * 0.22f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.01f), s * 0.24f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.01f), s * 0.25f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.01f), s * 0.25f, k_tol);
}

TEST_P(MotorControlDirectionTest, NoIntegrationWhileSaturated) {
  const float s = GetParam();
  Gains g;
  g.min_duty_left = 0.9f;
  g.kp = 0.5f;
  g.ki = 2.0f;
  g.integral_limit = 1.0f;
  configure(g);

  // ff(2) = 1.0 (clamped), + kp * 1 pushes the duty past the limit in the
  // direction of the error: the integral must not grow
  for (int i = 0; i < 5; ++i) {
    EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.01f), s * 1.0f,
                k_tol);
  }

  // once the wheel overshoots, a wound up integral would hold the duty up.
  // Without windup it is ff - kp * 1 = 0.5
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 3.0f, 0.01f), s * 0.5f, k_tol);
}

TEST_P(MotorControlDirectionTest, IntegratesWhileSaturatedAgainstError) {
  const float s = GetParam();
  Gains g;
  g.ki = 10.0f;
  g.integral_limit = 1.0f;
  configure(g);

  // ki * err * dt = 1, the integral jumps to its limit
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 1.0f, 0.1f), s * 0.2f, k_tol);

  // ff 0.2 + integral 1 saturates, but the error now points back. Integrating
  // is what unwinds the integral, so it has to continue: 1 - 10 * 0.5 * 0.1
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 2.5f, 0.1f), s * 1.0f, k_tol);
  EXPECT_NEAR(mc_.computeDutyLeft(s * 2.0f, s * 2.0f, 0.1f), s * 0.7f, k_tol);
}

INSTANTIATE_TEST_SUITE_P(BothDirections, MotorControlDirectionTest,
                         ::testing::Values(1.0f, -1.0f));

///////////////////////////////////////////////////////////////////////////////
// configure / reset
///////////////////////////////////////////////////////////////////////////////

TEST_F(MotorControlTest, ConfigureAppliesNewGains) {
  Gains g;
  g.kp = 0.5f;
  configure(g);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.7f, k_tol);

  g.min_duty_left = 0.1f;
  g.min_duty_right = 0.2f;
  g.slope_left = 0.2f;
  g.slope_right = 0.3f;
  g.kp = 0.1f;
  configure(g);
  // ff 0.1 + 0.2 * 2, + 0.1 * 1
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.6f, k_tol);
  // ff 0.2 + 0.3 * 2, + 0.1 * 1
  EXPECT_NEAR(mc_.computeDutyRight(2.0f, 1.0f, 0.01f), 0.9f, k_tol);
}

TEST_F(MotorControlTest, ConfigureAppliesNewIntegralGains) {
  configure(integralGains());
  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.22f, k_tol);

  Gains g = integralGains(0.03f);
  g.ki = 4.0f;
  configure(g);
  // 0.04 per call now, clamped at 0.03
  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.23f, k_tol);
}

TEST_F(MotorControlTest, ConfigureClearsIntegrals) {
  configure(integralGains());
  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  mc_.computeDutyRight(2.0f, 1.0f, 0.01f);

  // the firmware reconfigures on a live parameter change, an integral built
  // with the old gains would make the duty jump
  configure(integralGains());
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.20f, k_tol);
  EXPECT_NEAR(mc_.computeDutyRight(2.0f, 1.0f, 0.01f), 0.20f, k_tol);
}

TEST_F(MotorControlTest, ResetClearsIntegralsAndKeepsGains) {
  Gains g = integralGains();
  g.kp = 0.5f;
  configure(g);
  mc_.computeDutyLeft(2.0f, 1.0f, 0.01f);
  mc_.computeDutyRight(2.0f, 1.0f, 0.01f);

  // the watchdog resets the controller when commands stop arriving
  mc_.reset();
  // ff 0.2 + kp 0.5, no integral
  EXPECT_NEAR(mc_.computeDutyLeft(2.0f, 1.0f, 0.01f), 0.70f, k_tol);
  EXPECT_NEAR(mc_.computeDutyRight(2.0f, 1.0f, 0.01f), 0.70f, k_tol);
}

} // namespace
