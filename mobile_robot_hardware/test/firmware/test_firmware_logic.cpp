// host tests for the logic the firmware's loop() is built from: the periodic
// scheduler, the tick -> velocity conversion, the config update and the motor
// command watchdog. Compiled against test/arduino_stub/Arduino.h, whose clock
// wraps at 32 bit like the esp32's
#include "inc/Drive.hpp"
#include "inc/Scheduler.hpp"
#include "inc/Velocity.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace {

///////////////////////////////////////////////////////////////////////////////
// due()
///////////////////////////////////////////////////////////////////////////////

class SchedulerTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }
};

TEST_F(SchedulerTest, DueNotBeforeNext) {
  uint32_t next = 1000;

  EXPECT_FALSE(due(999, next, 1000));
  EXPECT_EQ(next, 1000u);
}

TEST_F(SchedulerTest, DueFiresAtNextAndAdvancesByPeriod) {
  uint32_t next = 1000;

  EXPECT_TRUE(due(1000, next, 1000));
  EXPECT_EQ(next, 2000u);
  // once per period, a second call at the same time must not fire again
  EXPECT_FALSE(due(1000, next, 1000));
}

TEST_F(SchedulerTest, DueLateWithinOnePeriodKeepsThePhase) {
  uint32_t next = 1000;

  // a loop() pass that was a bit late must not shift the following runs,
  // otherwise the control rate drifts below the configured one
  EXPECT_TRUE(due(1300, next, 1000));
  EXPECT_EQ(next, 2000u);
}

TEST_F(SchedulerTest, DueAfterLongStallFiresOnceAndDropsMissedRuns) {
  uint32_t next = 1000;

  // stalled for several periods (e.g. a blocking serial write). The missed
  // control runs are not caught up in a burst: it fires once and the schedule
  // restarts one period after now
  EXPECT_TRUE(due(5500, next, 1000));
  EXPECT_EQ(next, 6500u);
  EXPECT_FALSE(due(5500, next, 1000));
}

TEST_F(SchedulerTest, DueStallOfExactlyOnePeriodCatchesUpOnce) {
  uint32_t next = 1000;

  // boundary of the catch-up rule: a delay of exactly one period is not
  // "more than one period", so the missed run is caught up right away
  EXPECT_TRUE(due(2000, next, 1000));
  EXPECT_EQ(next, 2000u);
  EXPECT_TRUE(due(2000, next, 1000));
  EXPECT_EQ(next, 3000u);
  EXPECT_FALSE(due(2000, next, 1000));
}

TEST_F(SchedulerTest, DueNextWrapsPastZero) {
  // micros() wraps every ~71.6 min on the esp32, a robot running longer
  // crosses it. Here next lands just behind the wrap
  uint32_t next = 0xFFFFFE00u; // 2^32 - 512

  EXPECT_TRUE(due(0xFFFFFE00u, next, 1000));
  EXPECT_EQ(next, 488u);

  // still before the wrap: an unsigned compare would think next has passed
  EXPECT_FALSE(due(0xFFFFFF9Cu, next, 1000));
  EXPECT_FALSE(due(487, next, 1000));
  EXPECT_TRUE(due(488, next, 1000));
  EXPECT_EQ(next, 1488u);
}

TEST_F(SchedulerTest, DueStallAcrossTheWrapFiresOnce) {
  uint32_t next = 0xFFFFFF00u;

  // now is 3256 us past next, with the wrap in between
  EXPECT_TRUE(due(3000, next, 1000));
  EXPECT_EQ(next, 4000u);
}

TEST_F(SchedulerTest, DueWithTheStubClockAcrossTheWrap) {
  // the same pattern loop() uses: next derived from micros(), which wraps
  arduino_stub::setMicros(0xFFFFFFFFULL - 299);
  uint32_t next = micros();
  ASSERT_TRUE(due(micros(), next, 500));

  arduino_stub::advanceMicros(499);
  EXPECT_FALSE(due(micros(), next, 500));
  arduino_stub::advanceMicros(1);
  ASSERT_LT(micros(), 500UL); // the clock really wrapped
  EXPECT_TRUE(due(micros(), next, 500));
}

///////////////////////////////////////////////////////////////////////////////
// ticksToRad() / getVelocity()
///////////////////////////////////////////////////////////////////////////////

constexpr float k_ticksPerRev = 480.0f;
constexpr float k_twoPi = 2.0f * static_cast<float>(M_PI);
constexpr float k_velTol = 1e-4f;

TEST(Velocity, TicksToRadOneRevolution) {
  EXPECT_NEAR(ticksToRad(480, k_ticksPerRev), k_twoPi, k_velTol);
  EXPECT_NEAR(ticksToRad(-480, k_ticksPerRev), -k_twoPi, k_velTol);
  EXPECT_NEAR(ticksToRad(0, k_ticksPerRev), 0.0f, k_velTol);
}

TEST(Velocity, FromTicksAndDt) {
  // a tenth of a revolution in 100 ms is one revolution per second
  EXPECT_NEAR(getVelocity(148, 100, 100000, k_ticksPerRev), k_twoPi, k_velTol);
}

TEST(Velocity, NegativeWhenTicksDecrease) {
  EXPECT_NEAR(getVelocity(-148, -100, 100000, k_ticksPerRev), -k_twoPi,
              k_velTol);
}

TEST(Velocity, ZeroWithoutTicks) {
  EXPECT_NEAR(getVelocity(100, 100, 5000, k_ticksPerRev), 0.0f, k_velTol);
}

TEST(Velocity, ZeroWhenNoTimePassed) {
  // dividing by dt == 0 gave inf/NaN, which the PI turns into full duty (#30)
  const float vel = getVelocity(148, 100, 0, k_ticksPerRev);
  EXPECT_TRUE(std::isfinite(vel));
  EXPECT_NEAR(vel, 0.0f, k_velTol);
  EXPECT_NEAR(getVelocity(100, 100, 0, k_ticksPerRev), 0.0f, k_velTol);
}

TEST(Velocity, SingleMicrosecondDt) {
  // smallest dt, one tick: large but finite
  const float vel = getVelocity(1, 0, 1, k_ticksPerRev);
  EXPECT_TRUE(std::isfinite(vel));
  EXPECT_NEAR(vel, k_twoPi / k_ticksPerRev * 1e6f, 1.0f);
}

///////////////////////////////////////////////////////////////////////////////
// Drive: updateConfig() and the watchdog in updateMotor()
///////////////////////////////////////////////////////////////////////////////

// pins and resolution as in esp32_firmware.ino
constexpr uint8_t k_ena = 15, k_in1 = 2, k_in2 = 4;
constexpr uint8_t k_in3 = 16, k_in4 = 17, k_enb = 5;
constexpr uint8_t k_enc1a = 18, k_enc1b = 19;
constexpr uint8_t k_enc2a = 22, k_enc2b = 23;
constexpr uint8_t k_pwmRes = 16;
constexpr float k_dutyMax = 65535.0f;
// ledc channels used by L298N
constexpr uint8_t k_channelLeft = 1, k_channelRight = 2;

constexpr float k_dutyTol = 1e-4f;

class DriveTest : public ::testing::Test {
protected:
  void SetUp() override {
    arduino_stub::reset();
    motorDriver_.setup();
    encoderLeft_.setup();
    encoderRight_.setup();
  }

  // a config payload with no PI and slope 0.1, so a 2 rad/s command gives a
  // feedforward duty of 0.2
  static protocol::ConfigPayload defaultPayload() {
    protocol::ConfigPayload p;
    p.kp = 0.0f;
    p.ki = 0.0f;
    p.integral_limit = 1.0f;
    p.slope_left = 0.1f;
    p.slope_right = 0.1f;
    p.min_duty_left = 0.0f;
    p.min_duty_right = 0.0f;
    p.ticks_per_wheel_rev = k_ticksPerRev;
    p.watchdog_timeout_ms = 200;
    p.control_rate_hz = 200;
    p.report_rate_hz = 50;
    return p;
  }

  // the C line the pi would send, CRC included
  static std::string configLine(uint32_t id, const protocol::ConfigPayload &p) {
    std::string line =
        "C," + std::to_string(id) + "," + std::to_string(p.kp) + "," +
        std::to_string(p.ki) + "," + std::to_string(p.integral_limit) + "," +
        std::to_string(p.slope_left) + "," + std::to_string(p.slope_right) +
        "," + std::to_string(p.min_duty_left) + "," +
        std::to_string(p.min_duty_right) + "," +
        std::to_string(p.ticks_per_wheel_rev) + "," +
        std::to_string(p.watchdog_timeout_ms) + "," +
        std::to_string(p.control_rate_hz) + "," +
        std::to_string(p.report_rate_hz) + "," +
        std::to_string(p.invert_left ? 1 : 0) + "," +
        std::to_string(p.invert_right ? 1 : 0);
    char crc[8];
    snprintf(crc, sizeof(crc), ",%04X",
             protocol::crc16(line.c_str(), line.size()));
    return line + crc + "\n";
  }

  // receive a config over the fake serial and run updateConfig() on it, like
  // one loop() pass
  bool receiveConfig(uint32_t id, const protocol::ConfigPayload &p) {
    Serial.pushInput(configLine(id, p));
    sc_.read();
    return drive_.updateConfig();
  }

  // receive a motor command, it is timestamped with the current millis()
  void receiveCmd(const std::string &left, const std::string &right) {
    Serial.pushInput("M," + left + "," + right + "\n");
    sc_.read();
  }

  void updateMotor(int32_t leftTicks, int32_t rightTicks, uint32_t us) {
    EncoderSample e;
    e.left_ticks = leftTicks;
    e.right_ticks = rightTicks;
    e.us = us;
    drive_.updateMotor(e);
  }

  // normalized duty last written to a motor's pwm channel
  static float duty(uint8_t channel) {
    return arduino_stub::ledcChannel(channel).duty / k_dutyMax;
  }

  // IN pins both low with full duty: the L298 shorts the motor, it brakes
  static bool leftBraking() {
    return arduino_stub::digitalWritten(k_in1) == LOW &&
           arduino_stub::digitalWritten(k_in2) == LOW &&
           arduino_stub::ledcChannel(k_channelLeft).duty == 65535u;
  }
  static bool rightBraking() {
    return arduino_stub::digitalWritten(k_in3) == LOW &&
           arduino_stub::digitalWritten(k_in4) == LOW &&
           arduino_stub::ledcChannel(k_channelRight).duty == 65535u;
  }
  static bool leftForwards() {
    return arduino_stub::digitalWritten(k_in1) == HIGH &&
           arduino_stub::digitalWritten(k_in2) == LOW;
  }
  static bool rightForwards() {
    return arduino_stub::digitalWritten(k_in3) == HIGH &&
           arduino_stub::digitalWritten(k_in4) == LOW;
  }

  static bool logged(const std::string &message) {
    return Serial.output().find("L," + message + "\n") != std::string::npos;
  }

  SerialChannel sc_;
  L298N motorDriver_{k_ena, k_in1, k_in2, k_enb, k_in3, k_in4, k_pwmRes};
  Encoder encoderLeft_{k_enc1a, k_enc1b};
  Encoder encoderRight_{k_enc2a, k_enc2b};
  MotorControl motorController_;
  Drive drive_{sc_, motorDriver_, encoderLeft_, encoderRight_,
               motorController_};
};

// ---- updateConfig() ----

TEST_F(DriveTest, UpdateConfigFalseUntilAConfigArrives) {
  EXPECT_FALSE(drive_.updateConfig());
  EXPECT_FALSE(drive_.config().initialized);
}

TEST_F(DriveTest, UpdateConfigAppliesTheFirstConfig) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));

  EXPECT_TRUE(drive_.config().initialized);
  EXPECT_EQ(drive_.config().handshake_id, 1u);
  EXPECT_EQ(drive_.config().watchdog_timeout_ms, 200u);
  EXPECT_TRUE(logged("New configuration received"));
}

TEST_F(DriveTest, UpdateConfigSkipsAResendOfTheSameHandshake) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));
  receiveCmd("2.0", "2.0");
  updateMotor(0, 0, 0);
  ASSERT_TRUE(leftForwards());
  Serial.clearOutput();

  // the pi re-sends every 500 ms until it sees the ACK. Re-applying would
  // brake the driving robot each time
  EXPECT_FALSE(receiveConfig(1, defaultPayload()));
  EXPECT_TRUE(leftForwards());
  EXPECT_TRUE(rightForwards());
  EXPECT_NEAR(duty(k_channelLeft), 0.2f, k_dutyTol);
  EXPECT_FALSE(logged("New configuration received"));
  // the re-send is still ACKed, the pi waits for that
  EXPECT_NE(Serial.output().find("S,ACK,1\n"), std::string::npos);
}

TEST_F(DriveTest, UpdateConfigAppliesANewIdWithTheSamePayload) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));
  receiveCmd("2.0", "2.0");
  updateMotor(0, 0, 0);
  ASSERT_TRUE(leftForwards());

  EXPECT_TRUE(receiveConfig(2, defaultPayload()));
  EXPECT_EQ(drive_.config().handshake_id, 2u);
  EXPECT_TRUE(leftBraking());
  EXPECT_TRUE(rightBraking());
}

TEST_F(DriveTest, UpdateConfigAppliesAChangedPayloadWithTheSameId) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));

  // a restarted pi starts counting handshakes again and can reuse the id the
  // esp32 last applied, with other values
  protocol::ConfigPayload changed = defaultPayload();
  changed.slope_left = 0.2f;
  EXPECT_TRUE(receiveConfig(1, changed));
  EXPECT_FLOAT_EQ(drive_.config().slope_left, 0.2f);
}

TEST_F(DriveTest, UpdateConfigBrakesAndResetsTheIntegrators) {
  protocol::ConfigPayload p = defaultPayload();
  p.ki = 1.0f;
  ASSERT_TRUE(receiveConfig(1, p));
  receiveCmd("2.0", "2.0");

  // the wheels don't turn, so the integrators wind up: 1 * 2 rad/s * 0.1 s
  updateMotor(0, 0, 0);      // first sample: feedforward only
  updateMotor(0, 0, 100000); // 0.2 ff, integral 0 -> 0.2
  updateMotor(0, 0, 200000); // 0.2 ff + 0.2 integral
  ASSERT_NEAR(duty(k_channelLeft), 0.4f, k_dutyTol);

  ASSERT_TRUE(receiveConfig(2, p));
  EXPECT_TRUE(leftBraking());
  EXPECT_TRUE(rightBraking());

  // after the new config only the feedforward is left: no stale velocity is
  // used on the first sample and the integrators start from zero
  updateMotor(0, 0, 300000);
  EXPECT_NEAR(duty(k_channelLeft), 0.2f, k_dutyTol);
  EXPECT_NEAR(duty(k_channelRight), 0.2f, k_dutyTol);
  updateMotor(0, 0, 400000);
  EXPECT_NEAR(duty(k_channelLeft), 0.2f, k_dutyTol);
  EXPECT_NEAR(duty(k_channelRight), 0.2f, k_dutyTol);
}

TEST_F(DriveTest, UpdateConfigAppliesTheInversion) {
  protocol::ConfigPayload p = defaultPayload();
  p.invert_left = true;
  ASSERT_TRUE(receiveConfig(1, p));
  receiveCmd("2.0", "2.0");
  updateMotor(0, 0, 0);

  // the mirrored left motor turns the other way for "forwards"
  EXPECT_EQ(arduino_stub::digitalWritten(k_in1), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in2), HIGH);
  EXPECT_TRUE(rightForwards());

  // and its encoder counts the other way
  arduino_stub::setDigitalRead(k_enc1b, LOW);
  arduino_stub::setDigitalRead(k_enc2b, LOW);
  ASSERT_TRUE(arduino_stub::fireInterrupt(k_enc1a));
  ASSERT_TRUE(arduino_stub::fireInterrupt(k_enc2a));
  EXPECT_EQ(encoderLeft_.get_tick_count(), -1);
  EXPECT_EQ(encoderRight_.get_tick_count(), 1);
}

// ---- updateMotor() ----

TEST_F(DriveTest, FirstSampleUsesFeedforwardOnly) {
  protocol::ConfigPayload p = defaultPayload();
  p.kp = 0.5f;
  ASSERT_TRUE(receiveConfig(1, p));
  receiveCmd("2.0", "-2.0");

  // there is no earlier sample to get a velocity from
  updateMotor(1000, -1000, 123456);
  EXPECT_TRUE(leftForwards());
  EXPECT_NEAR(duty(k_channelLeft), 0.2f, k_dutyTol);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in3), LOW);
  EXPECT_EQ(arduino_stub::digitalWritten(k_in4), HIGH);
  EXPECT_NEAR(duty(k_channelRight), 0.2f, k_dutyTol);
}

TEST_F(DriveTest, ZeroCommandBrakesThatWheel) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));
  receiveCmd("0.0", "2.0");
  updateMotor(0, 0, 0);

  EXPECT_TRUE(leftBraking());
  EXPECT_TRUE(rightForwards());
}

TEST_F(DriveTest, VelocityAcrossTheMicrosWrap) {
  protocol::ConfigPayload p = defaultPayload();
  p.kp = 0.1f;
  ASSERT_TRUE(receiveConfig(1, p));
  // one revolution per second, what 48 ticks in 100 ms measure
  receiveCmd("6.283185", "6.283185");

  updateMotor(0, 0, 0xFFFFFFFFu - 49999u);
  // 100 ms later, micros() wrapped in between. With the right dt the measured
  // velocity matches the command and only the feedforward remains. A dt that
  // didn't wrap would measure ~0 rad/s and add kp * 6.28 on top
  updateMotor(48, 48, 50000);
  EXPECT_NEAR(duty(k_channelLeft), 0.1f * k_twoPi, 1e-3f);
  EXPECT_NEAR(duty(k_channelRight), 0.1f * k_twoPi, 1e-3f);
}

// ---- watchdog ----

TEST_F(DriveTest, WatchdogFollowsACommandUpToTheTimeout) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));
  arduino_stub::setMillis(1000);
  receiveCmd("2.0", "2.0");

  arduino_stub::advanceMillis(200);
  updateMotor(0, 0, 0);
  EXPECT_TRUE(leftForwards());
  EXPECT_TRUE(rightForwards());
  EXPECT_FALSE(logged("No new motor command received, stopping motors"));
}

TEST_F(DriveTest, WatchdogBrakesOnAStaleCommand) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));
  arduino_stub::setMillis(1000);
  receiveCmd("2.0", "2.0");
  updateMotor(0, 0, 0);
  ASSERT_TRUE(leftForwards());

  // the pi stopped sending (crashed, cable pulled): the robot must not keep
  // driving on the last command
  arduino_stub::advanceMillis(201);
  updateMotor(0, 0, 201000);
  EXPECT_TRUE(leftBraking());
  EXPECT_TRUE(rightBraking());
  EXPECT_TRUE(logged("No new motor command received, stopping motors"));
}

TEST_F(DriveTest, WatchdogResumesOnAFreshCommand) {
  protocol::ConfigPayload p = defaultPayload();
  p.ki = 1.0f;
  ASSERT_TRUE(receiveConfig(1, p));
  arduino_stub::setMillis(1000);
  receiveCmd("2.0", "2.0");
  updateMotor(0, 0, 0);
  updateMotor(0, 0, 100000);
  updateMotor(0, 0, 200000); // integrators wound up, see above
  ASSERT_NEAR(duty(k_channelLeft), 0.4f, k_dutyTol);

  arduino_stub::advanceMillis(300);
  updateMotor(0, 0, 300000);
  ASSERT_TRUE(leftBraking());

  receiveCmd("2.0", "2.0");
  updateMotor(0, 0, 400000);
  EXPECT_TRUE(leftForwards());
  EXPECT_TRUE(rightForwards());
  // restarts from the feedforward, the stop reset the integrators
  EXPECT_NEAR(duty(k_channelLeft), 0.2f, k_dutyTol);
  updateMotor(0, 0, 500000);
  EXPECT_NEAR(duty(k_channelLeft), 0.2f, k_dutyTol);
}

TEST_F(DriveTest, WatchdogAcrossTheMillisWrap) {
  ASSERT_TRUE(receiveConfig(1, defaultPayload()));
  // millis() wraps after ~49.7 days of uptime
  arduino_stub::setMillis(0xFFFFFFFFULL - 49);
  receiveCmd("2.0", "2.0");

  // 100 ms later, millis() is past the wrap: the command is still fresh
  arduino_stub::advanceMillis(100);
  ASSERT_LT(millis(), 100UL);
  updateMotor(0, 0, 0);
  EXPECT_TRUE(leftForwards());

  // and goes stale after the timeout as usual
  arduino_stub::advanceMillis(101);
  updateMotor(0, 0, 201000);
  EXPECT_TRUE(leftBraking());
}

} // namespace
