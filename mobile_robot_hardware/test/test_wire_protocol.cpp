#include "mobile_robot_hardware/wire_protocol.hpp"

// the firmware's parser, compiled against test/arduino_stub/Arduino.h
#include "inc/Messages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

using mobile_robot_hardware::parseHandshakeReply;
using mobile_robot_hardware::serializeConfig;

namespace {

// @return: true if `line` parses as a reply with `prefix`
bool parses(const std::string &line, const std::string &prefix,
            uint32_t *id = nullptr, std::string *rest = nullptr) {
  uint32_t parsedId = 0;
  std::string parsedRest;
  const bool ok = parseHandshakeReply(line, prefix, parsedId, parsedRest);
  if (id) {
    *id = parsedId;
  }
  if (rest) {
    *rest = parsedRest;
  }
  return ok;
}

} // namespace

TEST(ParseHandshakeReply, Ack) {
  uint32_t id = 0;
  std::string rest = "x";
  ASSERT_TRUE(parses("S,ACK,12345", "S,ACK,", &id, &rest));
  EXPECT_EQ(id, 12345u);
  EXPECT_EQ(rest, "");
}

TEST(ParseHandshakeReply, NackWithField) {
  uint32_t id = 0;
  std::string rest;
  ASSERT_TRUE(parses("S,NACK,42,slope_left", "S,NACK,", &id, &rest));
  EXPECT_EQ(id, 42u);
  EXPECT_EQ(rest, "slope_left");
}

TEST(ParseHandshakeReply, RejectsOtherPrefix) {
  EXPECT_FALSE(parses("S,NACK,42,kp", "S,ACK,"));
  EXPECT_FALSE(parses("S,SETUP", "S,ACK,"));
  EXPECT_FALSE(parses("L,S,ACK,1", "S,ACK,"));
}

TEST(ParseHandshakeReply, RejectsMissingOrMalformedId) {
  EXPECT_FALSE(parses("S,ACK,", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,,kp", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,abc", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,12x", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,1.5", "S,ACK,"));
}

TEST(ParseHandshakeReply, RejectsSignAndWhitespace) {
  EXPECT_FALSE(parses("S,ACK,-1", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,+1", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK, 1", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,1 ", "S,ACK,"));
}

TEST(ParseHandshakeReply, RangeIsUint32) {
  uint32_t id = 0;
  ASSERT_TRUE(parses("S,ACK,4294967295", "S,ACK,", &id));
  EXPECT_EQ(id, 4294967295u);

  // would alias id 0 and 1 after a cast to uint32_t
  EXPECT_FALSE(parses("S,ACK,4294967296", "S,ACK,"));
  EXPECT_FALSE(parses("S,ACK,4294967297", "S,ACK,"));
  EXPECT_FALSE(parses("S,NACK,18446744073709551617,kp", "S,NACK,"));
  EXPECT_FALSE(parses("S,ACK,99999999999999999999999", "S,ACK,"));
}

TEST(ParseHandshakeReply, AcceptsLeadingZeros) {
  uint32_t id = 0;
  ASSERT_TRUE(parses("S,ACK,007", "S,ACK,", &id));
  EXPECT_EQ(id, 7u);
}

namespace {

// what SerialChannel hands to msgs::Config::init(): the line without '\n'
std::string stripNewline(std::string line) {
  if (!line.empty() && line.back() == '\n') {
    line.pop_back();
  }
  return line;
}

void expectSamePayload(const protocol::ConfigPayload &a,
                       const protocol::ConfigPayload &b) {
  // exact comparison on purpose, the round trip must not lose any bits
  EXPECT_EQ(a.kp, b.kp);
  EXPECT_EQ(a.ki, b.ki);
  EXPECT_EQ(a.integral_limit, b.integral_limit);
  EXPECT_EQ(a.slope_left, b.slope_left);
  EXPECT_EQ(a.slope_right, b.slope_right);
  EXPECT_EQ(a.min_duty_left, b.min_duty_left);
  EXPECT_EQ(a.min_duty_right, b.min_duty_right);
  EXPECT_EQ(a.ticks_per_wheel_rev, b.ticks_per_wheel_rev);
  EXPECT_EQ(a.watchdog_timeout_ms, b.watchdog_timeout_ms);
  EXPECT_EQ(a.control_rate_hz, b.control_rate_hz);
  EXPECT_EQ(a.report_rate_hz, b.report_rate_hz);
  EXPECT_EQ(a.invert_left, b.invert_left);
  EXPECT_EQ(a.invert_right, b.invert_right);
}

// a valid payload with every field off its default
protocol::ConfigPayload samplePayload() {
  protocol::ConfigPayload p;
  p.kp = 0.35f;
  p.ki = 1.7f;
  p.integral_limit = 0.4f;
  p.slope_left = 0.0123f;
  p.slope_right = 0.0456f;
  p.min_duty_left = 0.18f;
  p.min_duty_right = 0.21f;
  p.ticks_per_wheel_rev = 1320.5f;
  p.watchdog_timeout_ms = 250;
  p.control_rate_hz = 400;
  p.report_rate_hz = 100;
  p.invert_left = true;
  p.invert_right = false;
  return p;
}

} // namespace

TEST(SerializeConfig, Format) {
  const std::string line = serializeConfig(samplePayload(), 123);
  EXPECT_EQ(line.rfind("C,123,", 0), 0u) << line;
  ASSERT_FALSE(line.empty());
  EXPECT_EQ(line.back(), '\n');
  EXPECT_EQ(std::count(line.begin(), line.end(), ','), 14) << line;
  // booleans as 0/1
  EXPECT_NE(line.find(",1,0\n"), std::string::npos) << line;
}

TEST(SerializeConfig, RoundTripThroughFirmwareParser) {
  const protocol::ConfigPayload sent = samplePayload();
  ASSERT_EQ(protocol::validateConfig(sent), protocol::CONFIG_OK);

  msgs::Config received;
  ASSERT_EQ(received.init(stripNewline(serializeConfig(sent, 4000000000u))), 0)
      << received.lastError();
  EXPECT_TRUE(received.initialized);
  EXPECT_EQ(received.timestamp_ms, 4000000000u);
  expectSamePayload(sent, received);
}

TEST(SerializeConfig, RoundTripDefaults) {
  const protocol::ConfigPayload sent;
  msgs::Config received;
  ASSERT_EQ(received.init(stripNewline(serializeConfig(sent, 1))), 0)
      << received.lastError();
  expectSamePayload(sent, received);
}

// values the pi accepts must not be rounded into values the esp32 rejects
TEST(SerializeConfig, RoundTripKeepsValuesNearLimits) {
  protocol::ConfigPayload sent = samplePayload();
  sent.kp = 1e-7f;
  sent.slope_left = 1e-7f; // must stay > 0
  sent.slope_right = std::nextafter(0.0f, 1.0f); // smallest positive float
  sent.min_duty_left = std::nextafter(1.0f, 0.0f); // must stay < 1
  sent.ticks_per_wheel_rev = 3.4e38f;
  ASSERT_EQ(protocol::validateConfig(sent), protocol::CONFIG_OK);

  msgs::Config received;
  ASSERT_EQ(received.init(stripNewline(serializeConfig(sent, 7))), 0)
      << received.lastError();
  expectSamePayload(sent, received);
}

TEST(FirmwareConfigParser, RejectsMalformedLines) {
  const std::string valid = stripNewline(serializeConfig(samplePayload(), 9));
  msgs::Config c;
  ASSERT_EQ(c.init(valid), 0);

  const std::string body = valid.substr(valid.find(',', 2)); // after the id
  const std::string withoutLastField = valid.substr(0, valid.rfind(','));

  for (const std::string &line : {
           std::string("C"),
           std::string("C,9"),
           std::string("X") + valid.substr(1), // wrong message type
           std::string("C,0") + body,          // id 0 is reserved
           std::string("C,-1") + body,
           std::string("C,abc") + body,
           std::string("C,4294967296") + body, // id above UINT32_MAX
           withoutLastField,                   // missing field
           valid + ",1",                       // extra field
           valid.substr(0, valid.size() - 1) + "2", // invert_right not 0/1
       }) {
    msgs::Config parsed;
    EXPECT_EQ(parsed.init(line), -1) << line;
    EXPECT_FALSE(parsed.initialized) << line;
    EXPECT_STREQ(parsed.lastError(), "malformed") << line;
  }
}

TEST(FirmwareConfigParser, NamesOutOfRangeFieldButKeepsId) {
  protocol::ConfigPayload p = samplePayload();
  p.slope_left = 0.0f;
  msgs::Config c;
  EXPECT_EQ(c.init(stripNewline(serializeConfig(p, 55))), -1);
  EXPECT_STREQ(c.lastError(), "slope_left");
  // the NACK names the handshake even though the config was rejected
  EXPECT_EQ(c.timestamp_ms, 55u);
}
