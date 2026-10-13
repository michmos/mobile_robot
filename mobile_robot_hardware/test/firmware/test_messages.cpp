// host tests for the firmware side of the message classes in Messages.hpp,
// compiled against test/arduino_stub/Arduino.h. Round trips with the pi side
// live in test_wire_protocol.cpp
#include "inc/Messages.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

class MessagesTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }
};

///////////////////////////////////////////////////////////////////////////////
// MotorCmd
///////////////////////////////////////////////////////////////////////////////

TEST_F(MessagesTest, MotorCmdDefaultIsStoppedAtTimeZero) {
  const msgs::MotorCmd cmd;
  EXPECT_NEAR(cmd.left_vel_cmd, 0.0f, 1e-6f);
  EXPECT_NEAR(cmd.right_vel_cmd, 0.0f, 1e-6f);
  EXPECT_EQ(cmd.timestamp_ms, 0u);
}

TEST_F(MessagesTest, MotorCmdInitSetsTimestampFromMillis) {
  // the watchdog compares this against millis(), so it must be the arrival
  // time of the line
  arduino_stub::setMillis(1234);
  msgs::MotorCmd cmd;
  ASSERT_EQ(cmd.init("M,1,2"), 0);
  EXPECT_EQ(cmd.timestamp_ms, 1234u);
}

TEST_F(MessagesTest, MotorCmdInitTimestampTruncatesToWholeMillis) {
  arduino_stub::setMicros(5999);
  msgs::MotorCmd cmd;
  ASSERT_EQ(cmd.init("M,1,2"), 0);
  EXPECT_EQ(cmd.timestamp_ms, 5u);
}

TEST_F(MessagesTest, MotorCmdReinitRefreshesTimestamp) {
  msgs::MotorCmd cmd;
  arduino_stub::setMillis(100);
  ASSERT_EQ(cmd.init("M,1,2"), 0);
  arduino_stub::advanceMillis(50);
  ASSERT_EQ(cmd.init("M,1,2"), 0);
  EXPECT_EQ(cmd.timestamp_ms, 150u);
}

TEST_F(MessagesTest, MotorCmdConstructorSetsTimestampFromMillis) {
  arduino_stub::setMillis(77);
  const msgs::MotorCmd cmd(1.0f, -1.0f);
  EXPECT_EQ(cmd.timestamp_ms, 77u);
}

TEST_F(MessagesTest, MotorCmdInitParsesVelocities) {
  msgs::MotorCmd cmd;
  ASSERT_EQ(cmd.init("M,-1.5,0.25"), 0);
  EXPECT_NEAR(cmd.left_vel_cmd, -1.5f, 1e-6f);
  EXPECT_NEAR(cmd.right_vel_cmd, 0.25f, 1e-6f);
}

class MotorCmdMalformed : public ::testing::TestWithParam<std::string> {
protected:
  void SetUp() override { arduino_stub::reset(); }
};

TEST_P(MotorCmdMalformed, IsRejected) {
  msgs::MotorCmd cmd;
  EXPECT_EQ(cmd.init(GetParam()), -1);
}

INSTANTIATE_TEST_SUITE_P(
    MessagesTest, MotorCmdMalformed,
    ::testing::Values("",          // empty
                      "M",         // no values
                      "M,1",       // right value missing
                      "M,,2",      // empty field is skipped by strtok
                      "M,a,2",     // not a number
                      "M,1,2x",    // trailing garbage
                      "M,1,2 ",    // trailing space, SerialChannel trims it
                      "MM,1,2",    // prefix longer than one char
                      "X,1,2",     // other message type
                      "C,1,2"));   // config prefix

///////////////////////////////////////////////////////////////////////////////
// Event
///////////////////////////////////////////////////////////////////////////////

TEST_F(MessagesTest, EventSetupHasNoDetail) {
  EXPECT_EQ(msgs::Event(msgs::Event::SETUP).serialize(), "S,SETUP\n");
}

TEST_F(MessagesTest, EventAckCarriesHandshakeId) {
  EXPECT_EQ(msgs::Event(msgs::Event::ACK, "42").serialize(), "S,ACK,42\n");
}

TEST_F(MessagesTest, EventNackCarriesHandshakeIdAndField) {
  EXPECT_EQ(msgs::Event(msgs::Event::NACK, "4294967295,kp").serialize(),
            "S,NACK,4294967295,kp\n");
}

TEST_F(MessagesTest, EventEmptyDetailAddsNoComma) {
  // a trailing comma would be an empty field to the pi parser
  EXPECT_EQ(msgs::Event(msgs::Event::ACK, "").serialize(), "S,ACK\n");
}

///////////////////////////////////////////////////////////////////////////////
// Log
///////////////////////////////////////////////////////////////////////////////

TEST_F(MessagesTest, LogPrefixesMessage) {
  EXPECT_EQ(msgs::Log("Buffer overflow").serialize(), "L,Buffer overflow\n");
}

TEST_F(MessagesTest, LogKeepsCommasInMessage) {
  // logs quote received lines, which contain commas
  EXPECT_EQ(msgs::Log("Invalid line: X,1,2").serialize(),
            "L,Invalid line: X,1,2\n");
}

TEST_F(MessagesTest, LogEmptyMessage) {
  EXPECT_EQ(msgs::Log("").serialize(), "L,\n");
}

///////////////////////////////////////////////////////////////////////////////
// ImuData
///////////////////////////////////////////////////////////////////////////////

TEST_F(MessagesTest, ImuDefaultIsIdentityOrientationAtRest) {
  EXPECT_EQ(msgs::ImuData().serialize(),
            "I,0.000000,0.000000,0.000000,1.000000,"
            "0.000000,0.000000,0.000000,"
            "0.000000,0.000000,0.000000\n");
}

TEST_F(MessagesTest, ImuFieldsAreInWireOrder) {
  // binary exact values, so the expected text doesn't depend on rounding
  const msgs::ImuData imu(0.5f, -0.25f, 0.125f, 0.75f, 1.5f, -2.5f, 3.5f,
                          -4.0f, 5.0f, -9.5f);
  EXPECT_EQ(imu.serialize(), "I,0.500000,-0.250000,0.125000,0.750000,"
                             "1.500000,-2.500000,3.500000,"
                             "-4.000000,5.000000,-9.500000\n");
}

TEST_F(MessagesTest, ImuValuesAreRoundedToSixDecimals) {
  // std::to_string formats like %f, so anything below 5e-7 is sent as 0
  const msgs::ImuData imu(1e-7f, -1e-7f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                          0.0f, 1000000.0f);
  EXPECT_EQ(imu.serialize(), "I,0.000000,-0.000000,0.000000,1.000000,"
                             "0.000000,0.000000,0.000000,"
                             "0.000000,0.000000,1000000.000000\n");
}

} // namespace
