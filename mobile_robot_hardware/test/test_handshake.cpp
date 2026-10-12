#include "mobile_robot_hardware/handshake.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

using mobile_robot_hardware::Handshake;
using std::chrono::milliseconds;
using State = Handshake::State;

namespace {

constexpr milliseconds k_retry{500};
constexpr milliseconds k_timeout{10000};

Handshake::Clock::time_point at(int64_t ms) {
  return Handshake::Clock::time_point(milliseconds(ms));
}

class HandshakeTest : public ::testing::Test {
protected:
  Handshake hs{k_retry, k_timeout};
};

} // namespace

TEST_F(HandshakeTest, IdleUntilStarted) {
  EXPECT_EQ(hs.state(), State::IDLE);
  EXPECT_EQ(hs.id(), 0u);
  EXPECT_FALSE(hs.dueForSend(at(1000)));
}

TEST_F(HandshakeTest, StartIsPendingAndDueImmediately) {
  const uint32_t id = hs.start(at(1000));
  EXPECT_EQ(id, 1000u);
  EXPECT_EQ(hs.id(), id);
  EXPECT_EQ(hs.state(), State::PENDING);
  EXPECT_TRUE(hs.dueForSend(at(1000)));
}

TEST_F(HandshakeTest, ResendsEveryRetryInterval) {
  hs.start(at(1000));
  ASSERT_TRUE(hs.dueForSend(at(1000)));
  EXPECT_FALSE(hs.dueForSend(at(1000)));
  EXPECT_FALSE(hs.dueForSend(at(1499)));
  EXPECT_TRUE(hs.dueForSend(at(1500)));
  EXPECT_FALSE(hs.dueForSend(at(1999)));
  EXPECT_TRUE(hs.dueForSend(at(2000)));
}

TEST_F(HandshakeTest, FailsAfterTimeout) {
  hs.start(at(1000));
  EXPECT_TRUE(hs.dueForSend(at(10999)));
  EXPECT_EQ(hs.state(), State::PENDING);

  EXPECT_FALSE(hs.dueForSend(at(11000)));
  EXPECT_EQ(hs.state(), State::FAILED);
  EXPECT_NE(hs.failureReason().find("timed out"), std::string::npos);
  EXPECT_FALSE(hs.dueForSend(at(20000)));
}

TEST_F(HandshakeTest, MatchingAckConfigures) {
  const uint32_t id = hs.start(at(1000));
  hs.onAck(id);
  EXPECT_EQ(hs.state(), State::CONFIGURED);
  EXPECT_FALSE(hs.dueForSend(at(1000)));
  // no deadline once configured
  EXPECT_FALSE(hs.dueForSend(at(100000)));
  EXPECT_EQ(hs.state(), State::CONFIGURED);
}

// a NACK can only mean a corrupted line, so it is retried, not final
TEST_F(HandshakeTest, MatchingNackKeepsRetrying) {
  const uint32_t id = hs.start(at(1000));
  ASSERT_TRUE(hs.dueForSend(at(1000)));

  EXPECT_TRUE(hs.onNack(id, "kp"));
  EXPECT_EQ(hs.state(), State::PENDING);
  EXPECT_EQ(hs.id(), id);
  // on the normal retry interval, not immediately
  EXPECT_FALSE(hs.dueForSend(at(1100)));
  EXPECT_TRUE(hs.dueForSend(at(1500)));

  hs.onAck(id);
  EXPECT_EQ(hs.state(), State::CONFIGURED);
}

TEST_F(HandshakeTest, TimeoutAfterNackNamesField) {
  const uint32_t id = hs.start(at(1000));
  hs.onNack(id, "slope_left");
  EXPECT_FALSE(hs.dueForSend(at(11000)));
  EXPECT_EQ(hs.state(), State::FAILED);
  EXPECT_NE(hs.failureReason().find("timed out"), std::string::npos);
  EXPECT_NE(hs.failureReason().find("'slope_left'"), std::string::npos);
}

TEST_F(HandshakeTest, NewHandshakeForgetsOldNack) {
  hs.onNack(hs.start(at(1000)), "kp");
  hs.start(at(2000));
  EXPECT_FALSE(hs.dueForSend(at(12000)));
  EXPECT_EQ(hs.failureReason().find("kp"), std::string::npos);
}

TEST_F(HandshakeTest, IgnoresRepliesWithStaleId) {
  const uint32_t first = hs.start(at(1000));
  const uint32_t second = hs.start(at(2000));
  ASSERT_NE(first, second);

  hs.onAck(first);
  EXPECT_EQ(hs.state(), State::PENDING);
  EXPECT_FALSE(hs.onNack(first, "kp"));
  EXPECT_EQ(hs.state(), State::PENDING);

  hs.onAck(second);
  EXPECT_EQ(hs.state(), State::CONFIGURED);
}

TEST_F(HandshakeTest, IgnoresRepliesOutsidePending) {
  const uint32_t id = hs.start(at(1000));
  hs.onAck(id);
  EXPECT_FALSE(hs.onNack(id, "kp"));
  EXPECT_EQ(hs.state(), State::CONFIGURED);

  // IDLE
  Handshake idle{k_retry, k_timeout};
  idle.onAck(0);
  EXPECT_EQ(idle.state(), State::IDLE);
}

TEST_F(HandshakeTest, SetupWhileConfiguredStartsNewHandshake) {
  const uint32_t first = hs.start(at(1000));
  hs.onAck(first);

  EXPECT_TRUE(hs.onSetup(at(5000)));
  EXPECT_EQ(hs.state(), State::PENDING);
  EXPECT_NE(hs.id(), first);
  EXPECT_TRUE(hs.dueForSend(at(5000)));
}

TEST_F(HandshakeTest, SetupWhileFailedStartsNewHandshake) {
  hs.start(at(1000));
  ASSERT_FALSE(hs.dueForSend(at(11000)));
  ASSERT_EQ(hs.state(), State::FAILED);

  EXPECT_TRUE(hs.onSetup(at(5000)));
  EXPECT_EQ(hs.state(), State::PENDING);
  EXPECT_TRUE(hs.failureReason().empty());
}

TEST_F(HandshakeTest, SetupWhilePendingKeepsHandshake) {
  const uint32_t id = hs.start(at(1000));
  ASSERT_TRUE(hs.dueForSend(at(1000)));

  EXPECT_FALSE(hs.onSetup(at(1100)));
  EXPECT_EQ(hs.state(), State::PENDING);
  EXPECT_EQ(hs.id(), id);
}

TEST_F(HandshakeTest, SetupWhilePendingKeepsRetryInterval) {
  hs.start(at(1000));
  ASSERT_TRUE(hs.dueForSend(at(1000)));

  // a burst of SETUPs must not make the config due before the retry interval
  for (int64_t t = 1010; t < 1500; t += 10) {
    hs.onSetup(at(t));
    EXPECT_FALSE(hs.dueForSend(at(t))) << "at " << t << " ms";
  }
  EXPECT_TRUE(hs.dueForSend(at(1500)));
}

TEST_F(HandshakeTest, IdIsNeverZero) {
  // ms since the clock epoch truncated to 32 bit is 0
  EXPECT_EQ(hs.start(at(int64_t{1} << 32)), 1u);
  EXPECT_NE(hs.start(at(0)), 0u);
}

TEST_F(HandshakeTest, RestartInSameMillisecondGetsNewId) {
  const uint32_t first = hs.start(at(1000));
  const uint32_t second = hs.start(at(1000));
  EXPECT_NE(first, second);
}

// only avoiding the previous id gave 1000, 1001, 1000
TEST_F(HandshakeTest, IdsNeverRepeatWithinOneMillisecond) {
  EXPECT_EQ(hs.start(at(1000)), 1000u);
  EXPECT_EQ(hs.start(at(1000)), 1001u);
  EXPECT_EQ(hs.start(at(1000)), 1002u);
  // the clock catching up again doesn't reuse an id either
  EXPECT_EQ(hs.start(at(1001)), 1003u);
  EXPECT_EQ(hs.start(at(5000)), 5000u);
}

TEST_F(HandshakeTest, IdsKeepIncreasingWhenMillisecondsWrap) {
  const int64_t wrap = int64_t{1} << 32;
  EXPECT_EQ(hs.start(at(wrap - 10)), UINT32_MAX - 9);
  // truncated to 32 bit the time jumps back to 5
  EXPECT_EQ(hs.start(at(wrap + 5)), UINT32_MAX - 8);
}

TEST_F(HandshakeTest, IdBumpSkipsZeroOnWrap) {
  const int64_t max = UINT32_MAX;
  EXPECT_EQ(hs.start(at(max)), UINT32_MAX);
  // same millisecond again: bumping UINT32_MAX must not give 0
  EXPECT_EQ(hs.start(at(max)), 1u);
}
