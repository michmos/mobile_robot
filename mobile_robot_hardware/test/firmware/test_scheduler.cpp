// host tests for the firmware's periodic scheduler in Scheduler.hpp,
// compiled against test/arduino_stub/Arduino.h, whose clock wraps at 32 bit
// like the esp32's
#include "inc/Scheduler.hpp"

#include <Arduino.h>
#include <gtest/gtest.h>

#include <cstdint>

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

} // namespace
