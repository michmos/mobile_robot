// host tests for the firmware's line reader and message dispatch,
// compiled against test/arduino_stub/Arduino.h. The pi side is played by
// queueing bytes on the fake Serial, replies are read back from its output
#include "inc/SerialChannel.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

namespace {

// commands are short decimal literals, parsed exactly enough for this
constexpr float k_tol = 1e-6f;

// appends the checksum the pi would send for `body`
std::string sign(const std::string &body) {
  char crc[8];
  std::snprintf(crc, sizeof(crc), "%04X",
                protocol::crc16(body.c_str(), body.size()));
  return body + "," + crc;
}

// a config line that passes the CRC and range check, without line ending
std::string validConfig(uint32_t handshakeId, const std::string &kp = "0.5") {
  return sign("C," + std::to_string(handshakeId) + "," + kp +
              ",1,0.4,0.1,0.1,0.1,0.1,480,200,200,50,0,1");
}

class SerialChannelTest : public ::testing::Test {
protected:
  void SetUp() override { arduino_stub::reset(); }

  // calls read() until the fake uart is empty, like loop() does over several
  // passes. Bounded, so a read() that stops consuming fails instead of hanging
  void readAll() {
    for (int i = 0; i < 100 && Serial.available() > 0; ++i) {
      sc_.read();
    }
    ASSERT_EQ(Serial.available(), 0);
  }

  void receive(const std::string &bytes) {
    Serial.pushInput(bytes);
    readAll();
  }

  void expectMotorCmd(float left, float right) {
    EXPECT_NEAR(sc_.getMotorCmd().left_vel_cmd, left, k_tol);
    EXPECT_NEAR(sc_.getMotorCmd().right_vel_cmd, right, k_tol);
  }

  SerialChannel sc_;
};

///////////////////////////////////////////////////////////////////////////////
// read(): line assembly
///////////////////////////////////////////////////////////////////////////////

TEST_F(SerialChannelTest, ReadWithNothingAvailableReadsNothing) {
  EXPECT_EQ(sc_.read(), 0u);
  EXPECT_EQ(Serial.output(), "");
  expectMotorCmd(0.0f, 0.0f);
}

TEST_F(SerialChannelTest, ReadReturnsBytesConsumed) {
  Serial.pushInput("M,1,2\n");
  EXPECT_EQ(sc_.read(), 6u);
}

TEST_F(SerialChannelTest, ReadConsumesAtMostOneChunkPerCall) {
  // loop() must not stall on a flood of input, read() takes 128 bytes a pass
  Serial.pushInput(std::string(200, '\n'));
  EXPECT_EQ(sc_.read(), 128u);
  EXPECT_EQ(Serial.available(), 72);
  EXPECT_EQ(sc_.read(), 72u);
}

TEST_F(SerialChannelTest, LineSplitOverSeveralChunksIsAssembled) {
  // the uart hands over whatever arrived so far, often mid line
  Serial.pushInput("M,1.");
  sc_.read();
  expectMotorCmd(0.0f, 0.0f);

  Serial.pushInput("5,-2");
  sc_.read();
  expectMotorCmd(0.0f, 0.0f);

  Serial.pushInput(".25\n");
  sc_.read();
  expectMotorCmd(1.5f, -2.25f);
  EXPECT_EQ(Serial.output(), "");
}

TEST_F(SerialChannelTest, LineSplitAcrossChunkBoundaryIsAssembled) {
  // 125 empty lines put the motor command across the 128 byte read chunk
  Serial.pushInput(std::string(125, '\n') + "M,3,4\n");
  sc_.read();
  expectMotorCmd(0.0f, 0.0f);
  sc_.read();
  expectMotorCmd(3.0f, 4.0f);
}

TEST_F(SerialChannelTest, SeveralLinesInOneChunkAreAllHandled) {
  receive(validConfig(1) + "\n" + validConfig(2) + "\nM,3,4\n");

  EXPECT_EQ(Serial.output(), "S,ACK,1\nS,ACK,2\n");
  EXPECT_EQ(sc_.getConfig().handshake_id, 2u);
  expectMotorCmd(3.0f, 4.0f);
}

TEST_F(SerialChannelTest, LfLineEndingIsTrimmed) {
  // the checksum would not match if a line ending were left on the line
  receive(validConfig(5) + "\n");
  EXPECT_EQ(Serial.output(), "S,ACK,5\n");
}

TEST_F(SerialChannelTest, CrlfLineEndingIsTrimmed) {
  // a terminal or a pi side println() sends \r\n
  receive(validConfig(5) + "\r\n");
  EXPECT_EQ(Serial.output(), "S,ACK,5\n");
}

TEST_F(SerialChannelTest, TrailingSpacesAreTrimmed) {
  receive(validConfig(5) + " \r\n");
  EXPECT_EQ(Serial.output(), "S,ACK,5\n");
}

TEST_F(SerialChannelTest, EmptyLinesAreIgnored) {
  receive("\n\r\n \r\n\n");

  EXPECT_EQ(Serial.output(), "");
  expectMotorCmd(0.0f, 0.0f);
  EXPECT_FALSE(sc_.getConfig().initialized);
}

TEST_F(SerialChannelTest, PartialLineIsKeptUntilItsNewline) {
  receive("M,1,2");
  expectMotorCmd(0.0f, 0.0f);
  EXPECT_EQ(Serial.output(), "");

  receive("\n");
  expectMotorCmd(1.0f, 2.0f);
}

TEST_F(SerialChannelTest, PartialLineAtBufferLimitIsKept) {
  // 256 bytes is the longest partial line kept, the limit is exclusive
  receive(std::string(256, 'x'));
  EXPECT_EQ(Serial.output(), "");

  receive("\n");
  EXPECT_EQ(Serial.output(),
            "L,Invalid line: " + std::string(256, 'x') + "\n");
}

TEST_F(SerialChannelTest, OverlongLineIsDroppedAndBufferRecovers) {
  // noise without a newline, e.g. a baud rate mismatch, must not grow the
  // buffer forever
  receive(std::string(257, 'x'));
  EXPECT_EQ(Serial.output(), "L,Buffer overflow, clearing buffer\n");
  Serial.clearOutput();

  // the newline that ends the dropped line leaves an empty line, then the
  // next line parses normally
  receive("\nM,1,2\n");
  EXPECT_EQ(Serial.output(), "");
  expectMotorCmd(1.0f, 2.0f);
}

TEST_F(SerialChannelTest, OverlongValidLookingLineIsNotApplied) {
  // a motor command padded past the limit is dropped before it completes
  receive("M,1,2" + std::string(300, '0'));
  receive("\nM,3,4\n");
  expectMotorCmd(3.0f, 4.0f);
}

///////////////////////////////////////////////////////////////////////////////
// line dispatch
///////////////////////////////////////////////////////////////////////////////

TEST_F(SerialChannelTest, ValidConfigIsAckedAndStored) {
  receive(validConfig(42) + "\n");

  EXPECT_EQ(Serial.output(), "S,ACK,42\n");
  EXPECT_TRUE(sc_.getConfig().initialized);
  EXPECT_EQ(sc_.getConfig().handshake_id, 42u);
  EXPECT_NEAR(sc_.getConfig().kp, 0.5f, k_tol);
  EXPECT_TRUE(sc_.getConfig().invert_right);
}

TEST_F(SerialChannelTest, ResentConfigIsAckedAgain) {
  // the pi re-sends until it sees the ACK, so a lost ACK must be re-sent
  receive(validConfig(42) + "\n" + validConfig(42) + "\n");
  EXPECT_EQ(Serial.output(), "S,ACK,42\nS,ACK,42\n");
}

TEST_F(SerialChannelTest, ConfigWithBadChecksumIsNacked) {
  std::string line = validConfig(7);
  // flip a payload digit, the checksum no longer matches
  line[line.find(",480,") + 1] = '5';
  receive(line + "\n");

  EXPECT_EQ(Serial.output(),
            "L,Invalid config received: " + line + "\nS,NACK,7,checksum\n");
  EXPECT_FALSE(sc_.getConfig().initialized);
}

TEST_F(SerialChannelTest, ConfigOutOfRangeIsNackedWithField) {
  const std::string line = validConfig(7, "-0.5");
  receive(line + "\n");

  EXPECT_EQ(Serial.output(),
            "L,Invalid config received: " + line + "\nS,NACK,7,kp\n");
  EXPECT_FALSE(sc_.getConfig().initialized);
}

TEST_F(SerialChannelTest, MalformedConfigIsNackedWithIdZero) {
  // no handshake id to name, so the NACK carries 0, which the pi never uses
  receive("C\n");
  EXPECT_EQ(Serial.output(),
            "L,Invalid config received: C\nS,NACK,0,checksum\n");
}

TEST_F(SerialChannelTest, RejectedConfigKeepsPreviousOne) {
  receive(validConfig(1) + "\n");
  receive(validConfig(2, "-0.5") + "\n");

  EXPECT_TRUE(sc_.getConfig().initialized);
  EXPECT_EQ(sc_.getConfig().handshake_id, 1u);
}

TEST_F(SerialChannelTest, ValidMotorCmdUpdatesCommand) {
  arduino_stub::setMillis(1234);
  receive("M,-1.5,2.5\n");

  expectMotorCmd(-1.5f, 2.5f);
  // the watchdog runs off the time the command arrived
  EXPECT_EQ(sc_.getMotorCmd().timestamp_ms, 1234u);
  // motor commands are not acknowledged
  EXPECT_EQ(Serial.output(), "");
}

TEST_F(SerialChannelTest, InvalidMotorCmdIsLoggedAndKeepsPreviousOne) {
  arduino_stub::setMillis(10);
  receive("M,1,2\n");
  arduino_stub::setMillis(20);
  receive("M,abc,2\n");

  EXPECT_EQ(Serial.output(), "L,Invalid motor command: M,abc,2\n");
  expectMotorCmd(1.0f, 2.0f);
  // a garbled command must not feed the watchdog
  EXPECT_EQ(sc_.getMotorCmd().timestamp_ms, 10u);
}

TEST_F(SerialChannelTest, UnknownPrefixIsIgnored) {
  receive("X,1,2\nS,ACK,1\n");

  // only logged, no reply that the pi could take for a handshake answer
  EXPECT_EQ(Serial.output(),
            "L,Invalid line: X,1,2\nL,Invalid line: S,ACK,1\n");
  expectMotorCmd(0.0f, 0.0f);
  EXPECT_FALSE(sc_.getConfig().initialized);
}

///////////////////////////////////////////////////////////////////////////////
// write
///////////////////////////////////////////////////////////////////////////////

TEST_F(SerialChannelTest, WritelnSendsMessageWithSingleNewline) {
  // serialize() already ends in \n, writeln must not add a second one
  sc_.writeln(msgs::Event(msgs::Event::SETUP));
  EXPECT_EQ(Serial.output(), "S,SETUP\n");
}

TEST_F(SerialChannelTest, WriteSendsMessageUnchanged) {
  sc_.write(msgs::Log("hello"));
  EXPECT_EQ(Serial.output(), "L,hello\n");
}

} // namespace
