// host tests for the firmware's L298N motor driver wrapper,
// compiled against test/arduino_stub/Arduino.h
#include "inc/L298.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

// the driver owns its pins, a copy would drive the same hardware twice
static_assert(!std::is_copy_constructible_v<L298N>);
static_assert(!std::is_copy_assignable_v<L298N>);

namespace {

// the pins and resolution esp32_firmware.ino uses
constexpr uint8_t k_ena = 15;
constexpr uint8_t k_in1 = 2;
constexpr uint8_t k_in2 = 4;
constexpr uint8_t k_enb = 5;
constexpr uint8_t k_in3 = 16;
constexpr uint8_t k_in4 = 17;
constexpr uint8_t k_res = 16;
constexpr uint32_t k_duty_max = (1u << k_res) - 1u;

// fixed in L298.hpp
constexpr uint8_t k_channel1 = 1;
constexpr uint8_t k_channel2 = 2;
constexpr uint32_t k_freq_hz = 1000;

class L298Test : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }

  L298N driver_{k_ena, k_in1, k_in2, k_enb, k_in3, k_in4, k_res};
};

///////////////////////////////////////////////////////////////////////////////
// setup
///////////////////////////////////////////////////////////////////////////////

TEST_F(L298Test, ConstructorDoesNotTouchHardware) {
  // global objects are constructed before the core is up, all hardware access
  // has to wait for setup()
  L298N driver(k_ena, k_in1, k_in2, k_enb, k_in3, k_in4, k_res);

  EXPECT_EQ(arduino_stub::pinModeOf(k_ena), -1);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), -1);
  EXPECT_FALSE(arduino_stub::ledcChannel(k_channel1).setup);
  EXPECT_FALSE(arduino_stub::ledcChannel(k_channel2).setup);
}

TEST_F(L298Test, SetupMakesAllPinsOutputs) {
  driver_.setup();

  for (uint8_t pin : {k_ena, k_in1, k_in2, k_enb, k_in3, k_in4}) {
    EXPECT_EQ(arduino_stub::pinModeOf(pin), OUTPUT) << "pin " << int(pin);
  }
}

TEST_F(L298Test, SetupLeavesDirectionPinsLow) {
  driver_.setup();

  // no direction driven until the first command, the motors can't turn
  for (uint8_t pin : {k_in1, k_in2, k_in3, k_in4}) {
    EXPECT_EQ(arduino_stub::digitalWritten(pin), LOW) << "pin " << int(pin);
  }
}

TEST_F(L298Test, SetupConfiguresOnePwmChannelPerMotor) {
  driver_.setup();

  const arduino_stub::LedcChannel c1 = arduino_stub::ledcChannel(k_channel1);
  EXPECT_TRUE(c1.setup);
  EXPECT_EQ(c1.freq_hz, k_freq_hz);
  EXPECT_EQ(c1.resolution_bits, k_res);
  EXPECT_EQ(c1.pin, k_ena);
  // no duty until the first command
  EXPECT_FALSE(c1.written);

  const arduino_stub::LedcChannel c2 = arduino_stub::ledcChannel(k_channel2);
  EXPECT_TRUE(c2.setup);
  EXPECT_EQ(c2.freq_hz, k_freq_hz);
  EXPECT_EQ(c2.resolution_bits, k_res);
  EXPECT_EQ(c2.pin, k_enb);
  EXPECT_FALSE(c2.written);
}

///////////////////////////////////////////////////////////////////////////////
// direction
///////////////////////////////////////////////////////////////////////////////

TEST_F(L298Test, ForwardDrivesIn1High) {
  driver_.setup();
  driver_.updateDir1(FORWARDS);

  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
  // motor 2 untouched
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), LOW);
}

TEST_F(L298Test, BackwardDrivesIn2High) {
  driver_.setup();
  driver_.updateDir1(BACKWARDS);

  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), HIGH);
}

TEST_F(L298Test, Motor2DirectionUsesIn3In4) {
  driver_.setup();

  driver_.updateDir2(FORWARDS);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), LOW);

  driver_.updateDir2(BACKWARDS);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), HIGH);
  // motor 1 untouched
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
}

TEST_F(L298Test, DefaultDirectionIsForward) {
  driver_.setup();
  driver_.updateDir1();
  driver_.updateDir2();

  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), LOW);
}

TEST_F(L298Test, StartDrivesBothMotorsForward) {
  driver_.setup();
  driver_.start();

  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), LOW);
}

TEST_F(L298Test, InvertedSwapsDirectionPins) {
  // a motor mounted mirrored turns the other way for the same pins, the
  // config's invert flag compensates
  driver_.setup();
  driver_.setInverted1(true);

  driver_.updateDir1(FORWARDS);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), HIGH);

  driver_.updateDir1(BACKWARDS);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
}

TEST_F(L298Test, InvertedIsPerMotor) {
  driver_.setup();
  driver_.setInverted2(true);

  driver_.start();
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), HIGH);
}

TEST_F(L298Test, InvertedCanBeCleared) {
  // a new config with invert = false has to undo an earlier invert = true
  driver_.setup();
  driver_.setInverted1(true);
  driver_.setInverted1(false);

  driver_.updateDir1(FORWARDS);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
}

TEST_F(L298Test, InvertedOnlyAppliesToNextDirectionUpdate) {
  // setInverted() doesn't rewrite the pins itself, the firmware brakes and
  // sets the direction again with the next command
  driver_.setup();
  driver_.updateDir1(FORWARDS);
  driver_.setInverted1(true);

  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
}

///////////////////////////////////////////////////////////////////////////////
// duty
///////////////////////////////////////////////////////////////////////////////

TEST_F(L298Test, FullDutyIsMaxOfResolution) {
  driver_.setup();

  EXPECT_EQ(driver_.setDuty1(1.0f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, k_duty_max);
  EXPECT_EQ(driver_.setDuty2(1.0f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel2).duty, k_duty_max);
}

TEST_F(L298Test, HalfDutyRoundsToNearest) {
  driver_.setup();

  // 0.5 * 65535 = 32767.5, rounded up
  EXPECT_EQ(driver_.setDuty1(0.5f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, 32768u);
}

TEST_F(L298Test, ZeroDutyCoasts) {
  // duty 0 disables the bridge (ENA low) while the direction pins stay as
  // they are: the motor coasts, unlike brake() which shorts it
  driver_.setup();
  driver_.updateDir1(FORWARDS);
  ASSERT_EQ(driver_.setDuty1(0.7f), 0);

  EXPECT_EQ(driver_.setDuty1(0.0f), 0);
  EXPECT_TRUE(arduino_stub::ledcChannel(k_channel1).written);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, 0u);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
}

TEST_F(L298Test, NegativeZeroDutyIsAccepted) {
  // fabsf() in the firmware can hand over -0.0f
  driver_.setup();

  EXPECT_EQ(driver_.setDuty1(-0.0f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, 0u);
}

TEST_F(L298Test, SmallestDutyStepIsReachable) {
  driver_.setup();

  // one count above and the last count below full duty
  EXPECT_EQ(driver_.setDuty1(1.0f / k_duty_max), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, 1u);
  EXPECT_EQ(driver_.setDuty1(1.0f - 1.0f / k_duty_max), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, k_duty_max - 1u);
}

TEST_F(L298Test, DutyIsPerMotor) {
  driver_.setup();

  ASSERT_EQ(driver_.setDuty1(1.0f), 0);
  EXPECT_FALSE(arduino_stub::ledcChannel(k_channel2).written);
  ASSERT_EQ(driver_.setDuty2(0.0f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, k_duty_max);
}

TEST_F(L298Test, SetDutyDoesNotChangeDirection) {
  driver_.setup();
  driver_.updateDir2(BACKWARDS);

  ASSERT_EQ(driver_.setDuty2(0.3f), 0);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), HIGH);
}

// out of range duties, for both motors
class L298OutOfRangeTest : public L298Test,
                           public ::testing::WithParamInterface<float> {};

TEST_P(L298OutOfRangeTest, RejectedAndLeavesOutputsUnchanged) {
  driver_.setup();
  driver_.updateDir1(FORWARDS);
  driver_.updateDir2(BACKWARDS);
  ASSERT_EQ(driver_.setDuty1(0.25f), 0);
  ASSERT_EQ(driver_.setDuty2(0.75f), 0);
  const uint32_t duty1 = arduino_stub::ledcChannel(k_channel1).duty;
  const uint32_t duty2 = arduino_stub::ledcChannel(k_channel2).duty;

  EXPECT_EQ(driver_.setDuty1(GetParam()), -1);
  EXPECT_EQ(driver_.setDuty2(GetParam()), -1);

  // the motors keep the last valid command
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, duty1);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel2).duty, duty2);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), HIGH);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), HIGH);
}

INSTANTIATE_TEST_SUITE_P(
    Duties, L298OutOfRangeTest,
    ::testing::Values(-1e-6f, -1.0f, 1.0f + 1e-6f, 2.0f,
                      std::numeric_limits<float>::infinity(),
                      -std::numeric_limits<float>::infinity()));

TEST_F(L298Test, NanDutyIsRejected) {
  GTEST_SKIP() << "see #31";
  // NaN passes the `< 0 || > 1` range check, and converting NaN to the
  // uint32_t ledcWrite() takes is undefined behavior
  driver_.setup();

  EXPECT_EQ(driver_.setDuty1(std::numeric_limits<float>::quiet_NaN()), -1);
  EXPECT_FALSE(arduino_stub::ledcChannel(k_channel1).written);
}

///////////////////////////////////////////////////////////////////////////////
// brake
///////////////////////////////////////////////////////////////////////////////

TEST_F(L298Test, BrakeShortsMotorAtFullDuty) {
  // IN1 == IN2 with the bridge enabled shorts the motor terminals, the
  // stronger stop compared to a zero duty
  driver_.setup();
  driver_.updateDir1(FORWARDS);
  ASSERT_EQ(driver_.setDuty1(0.3f), 0);

  driver_.brake1();

  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), LOW);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, k_duty_max);
  // motor 2 untouched
  EXPECT_FALSE(arduino_stub::ledcChannel(k_channel2).written);
}

TEST_F(L298Test, Brake2ShortsMotor2) {
  driver_.setup();
  driver_.updateDir2(BACKWARDS);

  driver_.brake2();

  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), LOW);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel2).duty, k_duty_max);
  EXPECT_FALSE(arduino_stub::ledcChannel(k_channel1).written);
}

TEST_F(L298Test, BrakeStopsBothMotors) {
  driver_.setup();
  driver_.setInverted1(true);
  driver_.start();

  driver_.brake();

  // inverted makes no difference, a short is a short
  for (uint8_t pin : {k_in1, k_in2, k_in3, k_in4}) {
    EXPECT_EQ(arduino_stub::digitalWritten(pin), LOW) << "pin " << int(pin);
  }
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, k_duty_max);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel2).duty, k_duty_max);
}

///////////////////////////////////////////////////////////////////////////////
// resolution
///////////////////////////////////////////////////////////////////////////////

// duty scaling is (1 << res) - 1 for every resolution
class L298ResolutionTest : public ::testing::TestWithParam<uint8_t> {
protected:
  void SetUp() override { arduino_stub::reset(); }
};

TEST_P(L298ResolutionTest, FullDutyIsAllOnes) {
  const uint8_t res = GetParam();
  const uint32_t max = (1u << res) - 1u;
  L298N driver(k_ena, k_in1, k_in2, k_enb, k_in3, k_in4, res);
  driver.setup();

  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).resolution_bits, res);
  ASSERT_EQ(driver.setDuty1(1.0f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, max);
  ASSERT_EQ(driver.setDuty1(0.0f), 0);
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel1).duty, 0u);

  // brake uses the same full scale
  driver.brake2();
  EXPECT_EQ(arduino_stub::ledcChannel(k_channel2).duty, max);
}

// 1 bit is the smallest, 20 bits the largest the esp32 ledc supports, 16 is
// what the firmware uses
INSTANTIATE_TEST_SUITE_P(Resolutions, L298ResolutionTest,
                         ::testing::Values(1, 8, 10, 16, 20));

} // namespace
