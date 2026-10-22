// component tests of how the hardware plugin turns the esp32's lines into
// joint states: "E," encoder reports become position and velocity, "S,SETUP"
// resets the tick baseline, "L," lines are logged and everything else is
// skipped. Driven with an active plugin against the fake esp32 on a pty, see
// hardware_fixture.hpp. Lifecycle and handshake are in
// test_mobile_robot_hardware.cpp

#include "hardware_fixture.hpp"

#include <gtest/gtest.h>

#include <rcutils/logging.h>

#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace mobile_robot_hardware::test;

namespace {

// set by the fixture instead of the xacro default, so the tick counts below
// are round fractions of a revolution
constexpr double k_ticks_per_rev = 480.0;

constexpr double k_tolerance = 1e-9;

double ticksToRad(double ticks) { return ticks * 2.0 * M_PI / k_ticks_per_rev; }

std::string encoderLine(int32_t left, int32_t right, uint32_t us) {
  return "E," + std::to_string(left) + "," + std::to_string(right) + "," +
         std::to_string(us);
}

// a changed value, NaN (never set) to a number counts as changed
bool changed(double before, double after) {
  if (std::isnan(before)) {
    return !std::isnan(after);
  }
  return before != after;
}

// records what is logged while it exists and passes it on to the previous
// output handler. The plugin logs from read(), which runs on the test thread
class LogCapture {
public:
  LogCapture() {
    previous_ = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&LogCapture::handle_);
  }
  ~LogCapture() { rcutils_logging_set_output_handler(previous_); }

  LogCapture(const LogCapture &) = delete;
  LogCapture &operator=(const LogCapture &) = delete;

  static bool contains(const std::string &text) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto &message : messages_) {
      if (message.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

private:
  static void handle_(const rcutils_log_location_t *location, int severity,
                      const char *name, rcutils_time_point_value_t timestamp,
                      const char *format, va_list *args) {
    va_list copy;
    va_copy(copy, *args);
    char buffer[512];
    std::vsnprintf(buffer, sizeof(buffer), format, copy);
    va_end(copy);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      messages_.emplace_back(buffer);
    }
    if (previous_ != nullptr) {
      previous_(location, severity, name, timestamp, format, args);
    }
  }

  static inline rcutils_logging_output_handler_t previous_ = nullptr;
  static inline std::mutex mutex_;
  static inline std::vector<std::string> messages_;
};

} // namespace

class EncoderStateTest : public MobileRobotHardwareTest {
protected:
  void SetUp() override {
    MobileRobotHardwareTest::SetUp();
    ASSERT_EQ(init(fakeParams({{"ticks_per_wheel_rev", "480"}})),
              CallbackReturn::SUCCESS);
    configureAndActivate();
    // the config line of on_configure, so waitForLine("C,") later only sees
    // a handshake started by the test
    fake_->discardReceived();
  }

  double stateValue(const std::string &name) const {
    for (const auto &iface : states_) {
      if (iface->get_name() == name) {
        const auto value = iface->get_optional<double>();
        EXPECT_TRUE(value.has_value()) << name;
        return value.value_or(std::nan(""));
      }
    }
    ADD_FAILURE() << "no state interface " << name;
    return std::nan("");
  }

  double leftPosition() const {
    return stateValue("left_wheel_joint/position");
  }
  double rightPosition() const {
    return stateValue("right_wheel_joint/position");
  }
  double leftVelocity() const {
    return stateValue("left_wheel_joint/velocity");
  }
  double rightVelocity() const {
    return stateValue("right_wheel_joint/velocity");
  }

  // calls read() until `done` holds. read() does not block and the lines
  // arrive asynchronously, so a line may need a few calls to show up
  // @return: false on timeout
  bool readUntil(const std::function<bool()> &done) {
    const auto deadline = std::chrono::steady_clock::now() + k_wait;
    while (std::chrono::steady_clock::now() < deadline) {
      EXPECT_EQ(hw_->read(now_, period_), return_type::OK);
      if (done()) {
        return true;
      }
      std::this_thread::yield();
    }
    return false;
  }

  // read() until the left position changes, i.e. an encoder report sent
  // since the last call has been processed. Every report a test sends moves
  // the left wheel, so this marks the point where it was used
  bool readUntilPositionChanges() {
    const double before = leftPosition();
    return readUntil([&] { return changed(before, leftPosition()); });
  }

  // sends an encoder report and read()s until it is used
  void feed(int32_t left, int32_t right, uint32_t us) {
    fake_->sendLine(encoderLine(left, right, us));
    ASSERT_TRUE(readUntilPositionChanges())
        << "report not processed: " << encoderLine(left, right, us);
  }

  // the esp32 restarts and is configured again: sends S,SETUP and read()s
  // until the plugin answers with a config line, which happens in the same
  // read() that handled the SETUP. The fake acks it on its own
  void restartEsp(const std::string &linesBeforeSetup = "") {
    fake_->sendRaw(linesBeforeSetup + "S,SETUP\n");
    ASSERT_TRUE(readUntilFakeReceives("C,").has_value());
  }
};

///////////////////////////////////////////////////////////////////////////////
// encoder reports
///////////////////////////////////////////////////////////////////////////////

// without a previous sample there is no time base for a velocity
TEST_F(EncoderStateTest, FirstSampleSetsPositionAndZeroVelocity) {
  feed(120, -240, 1000000);

  EXPECT_NEAR(leftPosition(), ticksToRad(120), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-240), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 0.0, k_tolerance);
  EXPECT_NEAR(rightVelocity(), 0.0, k_tolerance);
}

// the velocity uses the esp32's own timestamps, not the pi's read() period
TEST_F(EncoderStateTest, SecondSampleGivesVelocityFromEspTimestamps) {
  feed(100, -50, 1000000);
  // +-240 ticks (half a revolution) in 0.5 s
  feed(340, -290, 1500000);

  EXPECT_NEAR(leftPosition(), ticksToRad(340), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-290), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -2.0 * M_PI, 1e-6);
}

// micros() on the esp32 wraps after ~71.6 min of uptime
TEST_F(EncoderStateTest, TimestampWraparoundGivesPositiveDt) {
  // 0.5 s before the wrap
  feed(0, 0, UINT32_MAX - 499999);
  // 0.5 s after it, so dt is 1 s
  feed(480, -480, 500000);

  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -2.0 * M_PI, 1e-6);
}

// a repeated timestamp must not divide by zero. The velocity keeps its last
// value, the position still follows the ticks
TEST_F(EncoderStateTest, SameTimestampKeepsLastVelocity) {
  feed(0, 0, 1000000);
  feed(240, 240, 1500000);
  feed(480, 480, 1500000);

  EXPECT_NEAR(leftPosition(), ticksToRad(480), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(480), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), 2.0 * M_PI, 1e-6);
}

// same timestamp right after the first sample, before any velocity was set
TEST_F(EncoderStateTest, SameTimestampAfterFirstSampleKeepsZeroVelocity) {
  feed(0, 0, 1000000);
  feed(240, 240, 1000000);

  EXPECT_NEAR(leftPosition(), ticksToRad(240), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 0.0, k_tolerance);
  EXPECT_NEAR(rightVelocity(), 0.0, k_tolerance);
}

// driving backwards
TEST_F(EncoderStateTest, NegativeTickDeltaGivesNegativeVelocity) {
  feed(1000, -1000, 1000000);
  // left -240, right +240 ticks in 0.5 s
  feed(760, -760, 1500000);

  EXPECT_NEAR(leftPosition(), ticksToRad(760), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-760), k_tolerance);
  EXPECT_NEAR(leftVelocity(), -2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), 2.0 * M_PI, 1e-6);
}

// the plugin only looks at the newest report of a read() cycle. Using the
// older ones in between would give a velocity over 1.4..1.5 s instead
TEST_F(EncoderStateTest, OnlyNewestOfQueuedSamplesIsUsed) {
  feed(0, 0, 1000000);

  fake_->sendRaw(encoderLine(100, 100, 1100000) + "\n" +
                 encoderLine(200, 300, 1400000) + "\n" +
                 encoderLine(480, -480, 1500000) + "\n");
  ASSERT_TRUE(readUntilPositionChanges());

  EXPECT_NEAR(leftPosition(), ticksToRad(480), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-480), k_tolerance);
  // 480 ticks (one revolution) over the 0.5 s since the first sample
  EXPECT_NEAR(leftVelocity(), 4.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -4.0 * M_PI, 1e-6);
}

///////////////////////////////////////////////////////////////////////////////
// esp32 restart
///////////////////////////////////////////////////////////////////////////////

// a restarted esp32 counts from zero again, which must not look like the
// wheels jumped back by everything driven so far
TEST_F(EncoderStateTest, EspRestartKeepsPositionContinuous) {
  feed(1000, 2000, 1000000);
  feed(1240, 2240, 1500000);
  ASSERT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);

  restartEsp();
  // fresh tick counter and clock
  feed(120, 240, 200000);

  EXPECT_NEAR(leftPosition(), ticksToRad(1240 + 120), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(2240 + 240), k_tolerance);
  // the first sample after the restart is a new baseline, the old timestamp
  // means nothing to the restarted esp32's clock
  EXPECT_NEAR(leftVelocity(), 0.0, k_tolerance);
  EXPECT_NEAR(rightVelocity(), 0.0, k_tolerance);
}

TEST_F(EncoderStateTest, VelocityResumesAfterEspRestart) {
  feed(1000, 2000, 1000000);
  feed(1240, 2240, 1500000);
  restartEsp();
  feed(120, 240, 200000);

  feed(360, 0, 700000);

  EXPECT_NEAR(leftPosition(), ticksToRad(1240 + 360), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(2240), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -2.0 * M_PI, 1e-6);
}

// a report that was still buffered when the esp32 restarted belongs to its
// old tick counter. Used after the reset it would add those ticks again
TEST_F(EncoderStateTest, SamplesBeforeSetupInSameBatchAreDiscarded) {
  feed(100, 100, 1000000);

  restartEsp(encoderLine(5000, 5000, 1100000) + "\n");

  EXPECT_NEAR(leftPosition(), ticksToRad(100), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(100), k_tolerance);

  feed(10, 20, 50000);
  EXPECT_NEAR(leftPosition(), ticksToRad(110), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(120), k_tolerance);
}

///////////////////////////////////////////////////////////////////////////////
// other lines
///////////////////////////////////////////////////////////////////////////////

TEST_F(EncoderStateTest, LogLineIsLoggedAndKeepsState) {
  LogCapture log;
  feed(0, 0, 1000000);
  feed(240, -240, 1500000);

  fake_->sendLine("L,motor driver overheating");
  ASSERT_TRUE(readUntil(
      [] { return LogCapture::contains("motor driver overheating"); }));

  EXPECT_NEAR(leftPosition(), ticksToRad(240), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-240), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -2.0 * M_PI, 1e-6);
}

// lines the plugin can't use, e.g. after line noise or from mismatched
// firmware, are skipped without touching the state
class SkippedLineTest : public EncoderStateTest,
                        public ::testing::WithParamInterface<const char *> {};

TEST_P(SkippedLineTest, IsSkippedWithoutChangingState) {
  feed(0, 0, 1000000);
  feed(240, -240, 1500000);

  fake_->sendLine(GetParam());
  // a report sent after the line is only processed once the line was handled
  feed(480, -480, 2000000);

  EXPECT_NEAR(leftPosition(), ticksToRad(480), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-480), k_tolerance);
  // over the 0.5 s since the last valid report, so the skipped line didn't
  // move the time base either
  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -2.0 * M_PI, 1e-6);
}

INSTANTIATE_TEST_SUITE_P(
    EncoderState, SkippedLineTest,
    ::testing::Values("",                    // empty line
                      "garbage",             // no comma at all
                      "X,1,2,3",             // unknown type
                      "S,UNKNOWN",           // unknown status
                      "e,9000,9000,1700000", // lower case
                      "E,9000,9000",         // timestamp missing
                      "E,9000",              // right ticks missing
                      "E,",                  // nothing after the type
                      "E,,,",                // empty fields
                      "E,abc,9000,1700000",  // left ticks not a number
                      "E,9000,abc,1700000",  // right ticks not a number
                      "E,9000,9000,abc"));   // timestamp not a number

// the serial driver hands over whatever bytes arrived, a line can be split
// across two reads
TEST_F(EncoderStateTest, LineSplitAcrossTwoWritesIsReassembled) {
  // the first part of the next report arrives with the previous one
  fake_->sendRaw(encoderLine(0, 0, 1000000) + "\nE,240,-240,15");
  ASSERT_TRUE(readUntilPositionChanges());
  ASSERT_NEAR(leftPosition(), 0.0, k_tolerance);

  fake_->sendRaw("00000\n");
  ASSERT_TRUE(readUntilPositionChanges());

  EXPECT_NEAR(leftPosition(), ticksToRad(240), k_tolerance);
  EXPECT_NEAR(rightPosition(), ticksToRad(-240), k_tolerance);
  EXPECT_NEAR(leftVelocity(), 2.0 * M_PI, 1e-6);
  EXPECT_NEAR(rightVelocity(), -2.0 * M_PI, 1e-6);
}
