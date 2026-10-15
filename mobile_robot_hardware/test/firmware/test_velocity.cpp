// host tests for the firmware's tick -> velocity conversion in Velocity.hpp
#include "inc/Velocity.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace {

///////////////////////////////////////////////////////////////////////////////
// ticksToRad() / getVelocity()
///////////////////////////////////////////////////////////////////////////////

constexpr float k_ticksPerRev = 480.0f;
constexpr float k_twoPi = 2.0f * static_cast<float>(M_PI);
constexpr float k_velTol = 1e-4f;

TEST(VelocityTest, TicksToRadOneRevolution) {
  EXPECT_NEAR(ticksToRad(480, k_ticksPerRev), k_twoPi, k_velTol);
  EXPECT_NEAR(ticksToRad(-480, k_ticksPerRev), -k_twoPi, k_velTol);
  EXPECT_NEAR(ticksToRad(0, k_ticksPerRev), 0.0f, k_velTol);
}

TEST(VelocityTest, FromTicksAndDt) {
  // a tenth of a revolution in 100 ms is one revolution per second
  EXPECT_NEAR(getVelocity(148, 100, 100000, k_ticksPerRev), k_twoPi, k_velTol);
}

TEST(VelocityTest, NegativeWhenTicksDecrease) {
  EXPECT_NEAR(getVelocity(-148, -100, 100000, k_ticksPerRev), -k_twoPi,
              k_velTol);
}

TEST(VelocityTest, ZeroWithoutTicks) {
  EXPECT_NEAR(getVelocity(100, 100, 5000, k_ticksPerRev), 0.0f, k_velTol);
}

TEST(VelocityTest, ZeroWhenNoTimePassed) {
  // dividing by dt == 0 gave inf/NaN, which the PI turns into full duty (#30)
  const float vel = getVelocity(148, 100, 0, k_ticksPerRev);
  EXPECT_TRUE(std::isfinite(vel));
  EXPECT_NEAR(vel, 0.0f, k_velTol);
  EXPECT_NEAR(getVelocity(100, 100, 0, k_ticksPerRev), 0.0f, k_velTol);
}

TEST(VelocityTest, SingleMicrosecondDt) {
  // smallest dt, one tick: large but finite
  const float vel = getVelocity(1, 0, 1, k_ticksPerRev);
  EXPECT_TRUE(std::isfinite(vel));
  EXPECT_NEAR(vel, k_twoPi / k_ticksPerRev * 1e6f, 1.0f);
}

} // namespace
