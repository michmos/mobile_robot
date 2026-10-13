#include "mobile_robot_hardware/wire_protocol.hpp"

// the firmware's parser, compiled against test/arduino_stub/Arduino.h
#include "inc/Messages.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <cstdint>
#include <utility>
#include <vector>
#include <string>

using mobile_robot_hardware::parseEncoderReport;
using mobile_robot_hardware::parseHandshakeReply;
using mobile_robot_hardware::serializeMotorCmd;
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

// the config line without its ",<crc>" suffix
std::string bodyOf(const std::string &line) {
  const std::string stripped = stripNewline(line);
  return stripped.substr(0, stripped.rfind(','));
}

// appends the checksum the pi would send for `body`
std::string sign(const std::string &body) {
  char crc[8];
  std::snprintf(crc, sizeof(crc), "%04X",
                protocol::crc16(body.c_str(), body.size()));
  return body + "," + crc;
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

TEST(Crc16, CheckValue) {
  // the published check value of CRC-16/CCITT-FALSE
  EXPECT_EQ(protocol::crc16("123456789", 9), 0x29B1);
  EXPECT_EQ(protocol::crc16("", 0), 0xFFFF);
}

TEST(SerializeConfig, Format) {
  const std::string line = serializeConfig(samplePayload(), 123);
  EXPECT_EQ(line.rfind("C,123,", 0), 0u) << line;
  ASSERT_FALSE(line.empty());
  EXPECT_EQ(line.back(), '\n');
  EXPECT_EQ(std::count(line.begin(), line.end(), ','), 15) << line;
  // booleans as 0/1, then the checksum
  EXPECT_NE(line.find(",1,0,"), std::string::npos) << line;
  // CRC as 4 uppercase hex digits over everything before its comma
  EXPECT_EQ(stripNewline(line), sign(bodyOf(line))) << line;
}

TEST(SerializeConfig, RoundTripThroughFirmwareParser) {
  const protocol::ConfigPayload sent = samplePayload();
  ASSERT_EQ(protocol::validateConfig(sent), protocol::CONFIG_OK);

  msgs::Config received;
  ASSERT_EQ(received.init(stripNewline(serializeConfig(sent, 4000000000u))), 0)
      << received.lastError();
  EXPECT_TRUE(received.initialized);
  EXPECT_EQ(received.handshake_id, 4000000000u);
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

// lines with a correct checksum but broken content
TEST(FirmwareConfigParser, RejectsMalformedLines) {
  const std::string valid = stripNewline(serializeConfig(samplePayload(), 9));
  msgs::Config c;
  ASSERT_EQ(c.init(valid), 0);

  const std::string body = bodyOf(valid);
  const std::string afterId = body.substr(body.find(',', 2));
  const std::string withoutLastField = body.substr(0, body.rfind(','));

  for (const std::string &line : {
           sign("C"),
           sign("C,9"),
           sign("X" + body.substr(1)),          // wrong message type
           sign("C,0" + afterId),               // id 0 is reserved
           sign("C,-1" + afterId),
           sign("C,abc" + afterId),
           sign("C,4294967296" + afterId),      // id above UINT32_MAX
           sign(withoutLastField),              // missing field
           sign(body + ",1"),                   // extra field
           sign(body.substr(0, body.size() - 1) + "2"), // invert_right not 0/1
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
  EXPECT_EQ(c.handshake_id, 55u);
}

// a line damaged on the serial link must not be applied and ACKed
TEST(FirmwareConfigParser, RejectsCorruptedLines) {
  const std::string valid = stripNewline(serializeConfig(samplePayload(), 77));
  ASSERT_EQ(valid.rfind("C,77,0.349999994,", 0), 0u) << valid;

  std::string flippedDigit = valid;
  flippedDigit[8] = '8'; // kp 0.349999994 -> 0.389999994, still in range
  std::string droppedChar = valid;
  droppedChar.erase(9, 1);
  std::string badCrc = valid;
  badCrc.back() = badCrc.back() == '0' ? '1' : '0';

  for (const std::string &line : {
           flippedDigit,
           droppedChar,
           badCrc,
           std::string("C"),                      // no checksum at all
           bodyOf(valid),                         // checksum missing
           bodyOf(valid) + ",12",                 // too short
           bodyOf(valid) + ",+12F",               // not hex
           bodyOf(valid) + "," + valid.substr(valid.size() - 4) + "0",
       }) {
    msgs::Config parsed;
    EXPECT_EQ(parsed.init(line), -1) << line;
    EXPECT_FALSE(parsed.initialized) << line;
    EXPECT_STREQ(parsed.lastError(), "checksum") << line;
  }

  // the NACK still names the handshake, unless the id itself was hit
  msgs::Config parsed;
  parsed.init(flippedDigit);
  EXPECT_EQ(parsed.handshake_id, 77u);
}

TEST(FirmwareConfigParser, AcceptsLowercaseChecksum) {
  std::string line = stripNewline(serializeConfig(samplePayload(), 3));
  for (size_t i = line.rfind(',') + 1; i < line.size(); ++i) {
    line[i] = static_cast<char>(std::tolower(line[i]));
  }
  msgs::Config parsed;
  EXPECT_EQ(parsed.init(line), 0) << line << ": " << parsed.lastError();
}

TEST(SerializeMotorCmd, RoundTripThroughFirmwareParser) {
  // to_string's 6 fixed decimals would turn 1e-7 into 0 and cut 1.23456789
  for (const auto &cmd : {std::make_pair(0.0f, 0.0f),
                          std::make_pair(1.5f, -1.5f),
                          std::make_pair(1e-7f, -1.23456789f),
                          std::make_pair(-30.25f, 12345.678f)}) {
    const std::string line = serializeMotorCmd(cmd.first, cmd.second);
    ASSERT_EQ(line.back(), '\n');
    msgs::MotorCmd received;
    ASSERT_EQ(received.init(stripNewline(line)), 0) << line;
    EXPECT_EQ(received.left_vel_cmd, cmd.first) << line;
    EXPECT_EQ(received.right_vel_cmd, cmd.second) << line;
  }
}

// strtof() accepts these, but a velocity that isn't a finite number must be
// rejected like any other malformed command, not drive the wheel (#22)
TEST(FirmwareMotorCmdParser, RejectsNonFiniteVelocities) {
  for (const std::string &line : {
           std::string("M,nan,0"),
           std::string("M,0,nan"),
           std::string("M,NAN,0"),
           std::string("M,-nan,0"),
           std::string("M,inf,0"),
           std::string("M,0,-inf"),
           std::string("M,infinity,0"),
           std::string("M,1e39,0"), // overflows float to inf
       }) {
    msgs::MotorCmd parsed;
    EXPECT_EQ(parsed.init(line), -1) << line;
  }
}

TEST(FirmwareMotorCmdParser, AcceptsLargestFiniteVelocity) {
  msgs::MotorCmd parsed;
  ASSERT_EQ(parsed.init("M,3.4e38,-3.4e38"), 0);
  EXPECT_FLOAT_EQ(parsed.left_vel_cmd, 3.4e38f);
  EXPECT_FLOAT_EQ(parsed.right_vel_cmd, -3.4e38f);
}

// a config field can't be NaN or inf either. NaN already failed the range
// check, but an infinite gain passed it
TEST(FirmwareConfigParser, RejectsNonFiniteFloats) {
  const std::string valid = stripNewline(serializeConfig(samplePayload(), 9));
  const std::string body = bodyOf(valid);
  // C,9,<kp>,...: swap kp for a non-finite value
  const size_t kpStart = body.find(',', 2) + 1;
  const size_t kpEnd = body.find(',', kpStart);

  for (const char *value : {"nan", "inf", "-inf"}) {
    const std::string line =
        sign(body.substr(0, kpStart) + value + body.substr(kpEnd));
    msgs::Config parsed;
    EXPECT_EQ(parsed.init(line), -1) << line;
    EXPECT_FALSE(parsed.initialized) << line;
    EXPECT_STREQ(parsed.lastError(), "malformed") << line;
  }
}

TEST(ParseEncoderReport, RoundTripFromFirmware) {
  const int32_t lo = std::numeric_limits<int32_t>::min();
  const int32_t hi = std::numeric_limits<int32_t>::max();
  const msgs::EncoderData sent(lo, hi, 4294967295u);

  int32_t left = 0, right = 0;
  uint32_t us = 0;
  parseEncoderReport(stripNewline(sent.serialize()), left, right, us);
  EXPECT_EQ(left, lo);
  EXPECT_EQ(right, hi);
  EXPECT_EQ(us, 4294967295u);
}

TEST(ParseEncoderReport, RejectsMalformedLines) {
  int32_t left = 0, right = 0;
  uint32_t us = 0;
  for (const std::string line : {"", "E", "E,1,2", "X,1,2,3", "E,a,2,3",
                                 "E,1,,3", "E,1,2,"}) {
    EXPECT_THROW(parseEncoderReport(line, left, right, us),
                 std::runtime_error)
        << "'" << line << "'";
  }
}

// the esp32 skips a config only if id and payload match what it runs, so a
// pi restart that reuses an id with other values is still applied
TEST(SamePayload, ComparesEveryField) {
  const protocol::ConfigPayload base = samplePayload();
  EXPECT_TRUE(protocol::samePayload(base, base));

  std::vector<void (*)(protocol::ConfigPayload &)> changes = {
      [](protocol::ConfigPayload &p) { p.kp += 0.1f; },
      [](protocol::ConfigPayload &p) { p.ki += 0.1f; },
      [](protocol::ConfigPayload &p) { p.integral_limit += 0.1f; },
      [](protocol::ConfigPayload &p) { p.slope_left += 0.1f; },
      [](protocol::ConfigPayload &p) { p.slope_right += 0.1f; },
      [](protocol::ConfigPayload &p) { p.min_duty_left += 0.1f; },
      [](protocol::ConfigPayload &p) { p.min_duty_right += 0.1f; },
      [](protocol::ConfigPayload &p) { p.ticks_per_wheel_rev += 1.0f; },
      [](protocol::ConfigPayload &p) { ++p.watchdog_timeout_ms; },
      [](protocol::ConfigPayload &p) { ++p.control_rate_hz; },
      [](protocol::ConfigPayload &p) { ++p.report_rate_hz; },
      [](protocol::ConfigPayload &p) { p.invert_left = !p.invert_left; },
      [](protocol::ConfigPayload &p) { p.invert_right = !p.invert_right; },
  };
  for (size_t i = 0; i < changes.size(); ++i) {
    protocol::ConfigPayload changed = base;
    changes[i](changed);
    EXPECT_FALSE(protocol::samePayload(base, changed)) << "field " << i;
  }
}

TEST(SamePayload, SameIdDifferentValuesAfterRoundTrip) {
  protocol::ConfigPayload first = samplePayload();
  protocol::ConfigPayload second = first;
  second.kp = 0.5f;

  msgs::Config applied, resent;
  ASSERT_EQ(applied.init(stripNewline(serializeConfig(first, 1234))), 0);
  ASSERT_EQ(resent.init(stripNewline(serializeConfig(first, 1234))), 0);
  msgs::Config reused;
  ASSERT_EQ(reused.init(stripNewline(serializeConfig(second, 1234))), 0);

  // a plain re-send is skipped, a reused id with other values is not
  EXPECT_TRUE(protocol::samePayload(applied, resent));
  EXPECT_FALSE(protocol::samePayload(applied, reused));
}
