// the parts of the pi <-> esp32 wire contract that a round trip alone does not
// pin down: range limits, NACK details, line prefixes, the exact field order,
// the CRC and the edge cases of the firmware's number parsers.
// Firmware and plugin must always be updated together, these tests are what
// catches a change to only one side

#include "mobile_robot_hardware/wire_protocol.hpp"

// the firmware's side, compiled against test/arduino_stub/Arduino.h
#include "inc/Messages.hpp"

#include <gtest/gtest.h>

#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using mobile_robot_hardware::k_ack_prefix;
using mobile_robot_hardware::k_encoder_prefix;
using mobile_robot_hardware::k_log_prefix;
using mobile_robot_hardware::k_nack_prefix;
using mobile_robot_hardware::k_setup_line;
using mobile_robot_hardware::parseEncoderReport;
using mobile_robot_hardware::parseHandshakeReply;
using mobile_robot_hardware::serializeConfig;
using mobile_robot_hardware::serializeMotorCmd;

namespace {

constexpr float k_nan = std::numeric_limits<float>::quiet_NaN();
constexpr float k_inf = std::numeric_limits<float>::infinity();

// what SerialChannel hands to the parsers: the line without '\n'
std::string stripNewline(std::string line) {
  if (!line.empty() && line.back() == '\n') {
    line.pop_back();
  }
  return line;
}

// a valid payload with every field off its default
protocol::ConfigPayload validPayload() {
  protocol::ConfigPayload p;
  p.kp = 0.35f;
  p.ki = 1.5f;
  p.integral_limit = 0.25f;
  p.slope_left = 0.0125f;
  p.slope_right = 0.025f;
  p.min_duty_left = 0.1f;
  p.min_duty_right = 0.2f;
  p.ticks_per_wheel_rev = 1320.0f;
  p.watchdog_timeout_ms = 250;
  p.control_rate_hz = 400;
  p.report_rate_hz = 100;
  p.invert_motor_left = true;
  p.invert_motor_right = false;
  p.invert_encoder_left = false;
  p.invert_encoder_right = true;
  return p;
}

// appends the checksum the pi would send for `body`
std::string sign(const std::string &body) {
  char crc[8];
  std::snprintf(crc, sizeof(crc), "%04X",
                protocol::crc16(body.c_str(), body.size()));
  return body + "," + crc;
}

// splits a line at its commas, keeping empty fields
std::vector<std::string> splitFields(const std::string &line) {
  std::vector<std::string> fields;
  size_t start = 0;
  while (true) {
    const size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma - start));
    if (comma == std::string::npos) {
      return fields;
    }
    start = comma + 1;
  }
}

std::string joinFields(const std::vector<std::string> &fields) {
  std::string line;
  for (size_t i = 0; i < fields.size(); ++i) {
    line += (i == 0 ? "" : ",") + fields[i];
  }
  return line;
}

} // namespace

///////////////////////////////////////////////////////////////////////////////
// validateConfig boundaries and the NACK that names the field
///////////////////////////////////////////////////////////////////////////////

namespace {

struct BoundaryCase {
  const char *name;
  void (*change)(protocol::ConfigPayload &);
  protocol::e_config_field expected;
  // the literal the NACK carries for `expected`, spelled out so that renaming
  // a field in configFieldName() breaks this test
  const char *expectedName;
};

void PrintTo(const BoundaryCase &c, std::ostream *os) { *os << c.name; }

using P = protocol::ConfigPayload;
using protocol::CONFIG_OK;

// each case changes one field of validPayload(). Limits from ConfigPayload.hpp:
// gains >= 0, slopes > 0, 0 <= min duty < 1, ticks > 0,
// 1 <= watchdog <= 5000, 1 <= control rate <= 1000,
// 1 <= report rate <= control rate
const BoundaryCase k_boundary_cases[] = {
    // kp, ki, integral_limit: [0, inf)
    {"kp_min", [](P &p) { p.kp = 0.0f; }, CONFIG_OK, "ok"},
    // strtof("-0") gives -0.0f, which compares equal to 0
    {"kp_negative_zero", [](P &p) { p.kp = -0.0f; }, CONFIG_OK, "ok"},
    {"kp_below_min", [](P &p) { p.kp = -FLT_TRUE_MIN; }, protocol::CONFIG_KP,
     "kp"},
    {"kp_max", [](P &p) { p.kp = FLT_MAX; }, CONFIG_OK, "ok"},
    {"kp_nan", [](P &p) { p.kp = k_nan; }, protocol::CONFIG_KP, "kp"},
    {"kp_inf", [](P &p) { p.kp = k_inf; }, protocol::CONFIG_KP, "kp"},
    {"kp_minus_inf", [](P &p) { p.kp = -k_inf; }, protocol::CONFIG_KP, "kp"},
    {"ki_min", [](P &p) { p.ki = 0.0f; }, CONFIG_OK, "ok"},
    {"ki_below_min", [](P &p) { p.ki = -FLT_TRUE_MIN; }, protocol::CONFIG_KI,
     "ki"},
    {"ki_max", [](P &p) { p.ki = FLT_MAX; }, CONFIG_OK, "ok"},
    {"ki_nan", [](P &p) { p.ki = k_nan; }, protocol::CONFIG_KI, "ki"},
    {"ki_inf", [](P &p) { p.ki = k_inf; }, protocol::CONFIG_KI, "ki"},
    {"integral_limit_min", [](P &p) { p.integral_limit = 0.0f; }, CONFIG_OK,
     "ok"},
    {"integral_limit_below_min", [](P &p) { p.integral_limit = -FLT_TRUE_MIN; },
     protocol::CONFIG_INTEGRAL_LIMIT, "integral_limit"},
    {"integral_limit_max", [](P &p) { p.integral_limit = FLT_MAX; }, CONFIG_OK,
     "ok"},
    {"integral_limit_nan", [](P &p) { p.integral_limit = k_nan; },
     protocol::CONFIG_INTEGRAL_LIMIT, "integral_limit"},
    {"integral_limit_inf", [](P &p) { p.integral_limit = k_inf; },
     protocol::CONFIG_INTEGRAL_LIMIT, "integral_limit"},

    // slopes: (0, inf), the feedforward divides by them
    {"slope_left_min", [](P &p) { p.slope_left = FLT_TRUE_MIN; }, CONFIG_OK,
     "ok"},
    {"slope_left_zero", [](P &p) { p.slope_left = 0.0f; },
     protocol::CONFIG_SLOPE_LEFT, "slope_left"},
    {"slope_left_negative", [](P &p) { p.slope_left = -0.1f; },
     protocol::CONFIG_SLOPE_LEFT, "slope_left"},
    {"slope_left_max", [](P &p) { p.slope_left = FLT_MAX; }, CONFIG_OK, "ok"},
    {"slope_left_nan", [](P &p) { p.slope_left = k_nan; },
     protocol::CONFIG_SLOPE_LEFT, "slope_left"},
    {"slope_left_inf", [](P &p) { p.slope_left = k_inf; },
     protocol::CONFIG_SLOPE_LEFT, "slope_left"},
    {"slope_right_min", [](P &p) { p.slope_right = FLT_TRUE_MIN; }, CONFIG_OK,
     "ok"},
    {"slope_right_zero", [](P &p) { p.slope_right = 0.0f; },
     protocol::CONFIG_SLOPE_RIGHT, "slope_right"},
    {"slope_right_max", [](P &p) { p.slope_right = FLT_MAX; }, CONFIG_OK, "ok"},
    {"slope_right_nan", [](P &p) { p.slope_right = k_nan; },
     protocol::CONFIG_SLOPE_RIGHT, "slope_right"},
    {"slope_right_inf", [](P &p) { p.slope_right = k_inf; },
     protocol::CONFIG_SLOPE_RIGHT, "slope_right"},

    // min duty: [0, 1), a duty of 1 would leave no room for the PI term
    {"min_duty_left_min", [](P &p) { p.min_duty_left = 0.0f; }, CONFIG_OK,
     "ok"},
    {"min_duty_left_below_min", [](P &p) { p.min_duty_left = -FLT_TRUE_MIN; },
     protocol::CONFIG_MIN_DUTY_LEFT, "min_duty_left"},
    {"min_duty_left_max",
     [](P &p) { p.min_duty_left = std::nextafter(1.0f, 0.0f); }, CONFIG_OK,
     "ok"},
    {"min_duty_left_one", [](P &p) { p.min_duty_left = 1.0f; },
     protocol::CONFIG_MIN_DUTY_LEFT, "min_duty_left"},
    {"min_duty_left_nan", [](P &p) { p.min_duty_left = k_nan; },
     protocol::CONFIG_MIN_DUTY_LEFT, "min_duty_left"},
    {"min_duty_left_inf", [](P &p) { p.min_duty_left = k_inf; },
     protocol::CONFIG_MIN_DUTY_LEFT, "min_duty_left"},
    {"min_duty_right_min", [](P &p) { p.min_duty_right = 0.0f; }, CONFIG_OK,
     "ok"},
    {"min_duty_right_below_min", [](P &p) { p.min_duty_right = -FLT_TRUE_MIN; },
     protocol::CONFIG_MIN_DUTY_RIGHT, "min_duty_right"},
    {"min_duty_right_max",
     [](P &p) { p.min_duty_right = std::nextafter(1.0f, 0.0f); }, CONFIG_OK,
     "ok"},
    {"min_duty_right_one", [](P &p) { p.min_duty_right = 1.0f; },
     protocol::CONFIG_MIN_DUTY_RIGHT, "min_duty_right"},
    {"min_duty_right_nan", [](P &p) { p.min_duty_right = k_nan; },
     protocol::CONFIG_MIN_DUTY_RIGHT, "min_duty_right"},
    {"min_duty_right_minus_inf", [](P &p) { p.min_duty_right = -k_inf; },
     protocol::CONFIG_MIN_DUTY_RIGHT, "min_duty_right"},

    // ticks per revolution: (0, inf), the tick -> rad conversion divides by it
    {"ticks_per_wheel_rev_min",
     [](P &p) { p.ticks_per_wheel_rev = FLT_TRUE_MIN; }, CONFIG_OK, "ok"},
    {"ticks_per_wheel_rev_zero", [](P &p) { p.ticks_per_wheel_rev = 0.0f; },
     protocol::CONFIG_TICKS_PER_WHEEL_REV, "ticks_per_wheel_rev"},
    {"ticks_per_wheel_rev_max", [](P &p) { p.ticks_per_wheel_rev = FLT_MAX; },
     CONFIG_OK, "ok"},
    {"ticks_per_wheel_rev_nan", [](P &p) { p.ticks_per_wheel_rev = k_nan; },
     protocol::CONFIG_TICKS_PER_WHEEL_REV, "ticks_per_wheel_rev"},
    {"ticks_per_wheel_rev_inf", [](P &p) { p.ticks_per_wheel_rev = k_inf; },
     protocol::CONFIG_TICKS_PER_WHEEL_REV, "ticks_per_wheel_rev"},

    // watchdog: [1, 5000] ms
    {"watchdog_timeout_ms_zero", [](P &p) { p.watchdog_timeout_ms = 0; },
     protocol::CONFIG_WATCHDOG_TIMEOUT_MS, "watchdog_timeout_ms"},
    {"watchdog_timeout_ms_min", [](P &p) { p.watchdog_timeout_ms = 1; },
     CONFIG_OK, "ok"},
    {"watchdog_timeout_ms_max", [](P &p) { p.watchdog_timeout_ms = 5000; },
     CONFIG_OK, "ok"},
    {"watchdog_timeout_ms_above_max",
     [](P &p) { p.watchdog_timeout_ms = 5001; },
     protocol::CONFIG_WATCHDOG_TIMEOUT_MS, "watchdog_timeout_ms"},
    {"watchdog_timeout_ms_uint16_max",
     [](P &p) { p.watchdog_timeout_ms = UINT16_MAX; },
     protocol::CONFIG_WATCHDOG_TIMEOUT_MS, "watchdog_timeout_ms"},

    // control rate: [1, 1000] Hz, the report rate may not exceed it
    {"control_rate_hz_zero", [](P &p) { p.control_rate_hz = 0; },
     protocol::CONFIG_CONTROL_RATE_HZ, "control_rate_hz"},
    {"control_rate_hz_min",
     [](P &p) {
       p.control_rate_hz = 1;
       p.report_rate_hz = 1;
     },
     CONFIG_OK, "ok"},
    {"control_rate_hz_max", [](P &p) { p.control_rate_hz = 1000; }, CONFIG_OK,
     "ok"},
    {"control_rate_hz_above_max", [](P &p) { p.control_rate_hz = 1001; },
     protocol::CONFIG_CONTROL_RATE_HZ, "control_rate_hz"},

    // report rate: [1, control_rate_hz] Hz
    {"report_rate_hz_zero", [](P &p) { p.report_rate_hz = 0; },
     protocol::CONFIG_REPORT_RATE_HZ, "report_rate_hz"},
    {"report_rate_hz_min", [](P &p) { p.report_rate_hz = 1; }, CONFIG_OK, "ok"},
    {"report_rate_hz_equals_control_rate",
     [](P &p) { p.report_rate_hz = p.control_rate_hz; }, CONFIG_OK, "ok"},
    {"report_rate_hz_above_control_rate",
     [](P &p) { p.report_rate_hz = p.control_rate_hz + 1; },
     protocol::CONFIG_REPORT_RATE_HZ, "report_rate_hz"},
    {"report_rate_hz_at_rate_max",
     [](P &p) {
       p.control_rate_hz = 1000;
       p.report_rate_hz = 1000;
     },
     CONFIG_OK, "ok"},
};

class ConfigBoundary : public ::testing::TestWithParam<BoundaryCase> {
protected:
  protocol::ConfigPayload payload() const {
    protocol::ConfigPayload p = validPayload();
    GetParam().change(p);
    return p;
  }

  // the line carries "nan"/"inf" for a non-finite float
  static bool allFloatsFinite(const protocol::ConfigPayload &p) {
    return std::isfinite(p.kp) && std::isfinite(p.ki) &&
           std::isfinite(p.integral_limit) && std::isfinite(p.slope_left) &&
           std::isfinite(p.slope_right) && std::isfinite(p.min_duty_left) &&
           std::isfinite(p.min_duty_right) &&
           std::isfinite(p.ticks_per_wheel_rev);
  }
};

TEST_P(ConfigBoundary, ValidateConfigNamesField) {
  const protocol::e_config_field result = protocol::validateConfig(payload());
  EXPECT_EQ(result, GetParam().expected);
  EXPECT_STREQ(protocol::configFieldName(result), GetParam().expectedName);
}

// the esp32 reaches the same verdict on the line the pi sends, and its reply
// names the field in a form the pi parses. A non-finite float is already
// rejected by the firmware's parseFloat_ (#47), before the range check, so the
// esp32 NACKs it as "malformed" instead of naming the field
TEST_P(ConfigBoundary, FirmwareReplyNamesFieldToPi) {
  const uint32_t handshakeId = 31;
  msgs::Config received;
  received.init(stripNewline(serializeConfig(payload(), handshakeId)));
  const std::string reply =
      stripNewline(msgs::configReply(received).serialize());

  uint32_t id = 0;
  std::string detail;
  if (GetParam().expected == CONFIG_OK) {
    EXPECT_TRUE(received.initialized) << received.lastError();
    ASSERT_TRUE(parseHandshakeReply(reply, k_ack_prefix, id, detail)) << reply;
    EXPECT_EQ(detail, "");
  } else {
    EXPECT_FALSE(received.initialized);
    ASSERT_TRUE(parseHandshakeReply(reply, k_nack_prefix, id, detail)) << reply;
    EXPECT_EQ(detail, allFloatsFinite(payload()) ? GetParam().expectedName
                                                 : "malformed");
  }
  EXPECT_EQ(id, handshakeId);
}

INSTANTIATE_TEST_SUITE_P(
    Fields, ConfigBoundary, ::testing::ValuesIn(k_boundary_cases),
    [](const ::testing::TestParamInfo<BoundaryCase> &info) {
      return std::string(info.param.name);
    });

} // namespace

// the NACKs that do not name a field still reach the pi with their reason
TEST(ConfigReply, ChecksumNackParsedByPi) {
  std::string line = stripNewline(serializeConfig(validPayload(), 8));
  line.back() = line.back() == '0' ? '1' : '0';
  msgs::Config received;
  ASSERT_EQ(received.init(line), -1);

  uint32_t id = 0;
  std::string detail;
  ASSERT_TRUE(
      parseHandshakeReply(stripNewline(msgs::configReply(received).serialize()),
                          k_nack_prefix, id, detail));
  EXPECT_EQ(id, 8u);
  EXPECT_EQ(detail, "checksum");
}

// a malformed id is reported as id 0, which the pi never uses for a handshake,
// so the NACK cannot be mistaken for the reply to a running one
TEST(ConfigReply, MalformedIdNackParsedByPiAsIdZero) {
  std::vector<std::string> fields =
      splitFields(stripNewline(serializeConfig(validPayload(), 8)));
  fields.pop_back(); // the crc
  fields[1] = "x";
  msgs::Config received;
  ASSERT_EQ(received.init(sign(joinFields(fields))), -1);

  uint32_t id = 1;
  std::string detail;
  ASSERT_TRUE(
      parseHandshakeReply(stripNewline(msgs::configReply(received).serialize()),
                          k_nack_prefix, id, detail));
  EXPECT_EQ(id, 0u);
  EXPECT_EQ(detail, "malformed");
}

///////////////////////////////////////////////////////////////////////////////
// esp32 -> pi line prefixes
///////////////////////////////////////////////////////////////////////////////

namespace {

bool startsWith(const std::string &line, const std::string &prefix) {
  return line.rfind(prefix, 0) == 0;
}

} // namespace

// the pi compares the SETUP line as a whole, a detail would hide it
TEST(StatusLines, SetupIsExactlyWhatPiMatches) {
  EXPECT_EQ(stripNewline(msgs::Event(msgs::Event::SETUP).serialize()),
            k_setup_line);
}

TEST(StatusLines, AckStartsWithPiPrefix) {
  const std::string line = msgs::Event(msgs::Event::ACK, "5").serialize();
  EXPECT_TRUE(startsWith(line, k_ack_prefix)) << line;
  EXPECT_EQ(line.back(), '\n');
}

TEST(StatusLines, NackStartsWithPiPrefix) {
  const std::string line = msgs::Event(msgs::Event::NACK, "5,kp").serialize();
  EXPECT_TRUE(startsWith(line, k_nack_prefix)) << line;
  // "S,NACK," must not be taken for an ACK, the pi tries ACK first
  EXPECT_FALSE(startsWith(line, k_ack_prefix)) << line;
  EXPECT_EQ(line.back(), '\n');
}

TEST(StatusLines, LogStartsWithPiPrefix) {
  EXPECT_EQ(msgs::Log("motors stopped").serialize(),
            std::string(k_log_prefix) + "motors stopped\n");
}

TEST(StatusLines, EncoderDataStartsWithPiPrefix) {
  const std::string line = msgs::EncoderData(1, -2, 3).serialize();
  EXPECT_TRUE(startsWith(line, k_encoder_prefix)) << line;
  EXPECT_EQ(line.back(), '\n');
}

///////////////////////////////////////////////////////////////////////////////
// golden lines
///////////////////////////////////////////////////////////////////////////////

// swapping two fields of the same type would still round trip through both
// sides if both were changed, but break a firmware that was not reflashed
TEST(GoldenLines, ConfigFieldOrder) {
  EXPECT_EQ(serializeConfig(validPayload(), 123),
            "C,123,0.349999994,1.5,0.25,0.0125000002,0.0250000004,"
            "0.100000001,0.200000003,1320,250,400,100,1,0,0,1,6BB9\n");
}

TEST(GoldenLines, MotorCmdFieldOrder) {
  EXPECT_EQ(serializeMotorCmd(0.1f, -2.5f), "M,0.100000001,-2.5\n");
}

TEST(GoldenLines, EncoderDataFieldOrder) {
  EXPECT_EQ(msgs::EncoderData(-12, 34, 5678u).serialize(), "E,-12,34,5678\n");

  int32_t left = 0, right = 0;
  uint32_t us = 0;
  parseEncoderReport("E,-12,34,5678", left, right, us);
  EXPECT_EQ(left, -12);
  EXPECT_EQ(right, 34);
  EXPECT_EQ(us, 5678u);
}

TEST(GoldenLines, ConfigReplies) {
  msgs::Config accepted;
  ASSERT_EQ(accepted.init(stripNewline(serializeConfig(validPayload(), 77))),
            0);
  EXPECT_EQ(msgs::configReply(accepted).serialize(), "S,ACK,77\n");

  protocol::ConfigPayload bad = validPayload();
  bad.ki = -1.0f;
  msgs::Config rejected;
  ASSERT_EQ(rejected.init(stripNewline(serializeConfig(bad, 78))), -1);
  EXPECT_EQ(msgs::configReply(rejected).serialize(), "S,NACK,78,ki\n");
}

///////////////////////////////////////////////////////////////////////////////
// CRC
///////////////////////////////////////////////////////////////////////////////

namespace {

class CrcBitFlip : public ::testing::TestWithParam<size_t> {};

// CRC-16 detects every single bit error, so a flip anywhere in the checked
// part of the line must be rejected - including flips that leave a value in
// range or turn a digit into another digit
TEST_P(CrcBitFlip, RejectsEveryBitOfByte) {
  const std::string valid = stripNewline(serializeConfig(validPayload(), 123));
  const size_t pos = GetParam();
  ASSERT_LT(pos, valid.rfind(',')) << "position must be inside the body";

  for (int bit = 0; bit < 8; ++bit) {
    std::string corrupted = valid;
    corrupted[pos] = static_cast<char>(corrupted[pos] ^ (1 << bit));
    msgs::Config parsed;
    EXPECT_EQ(parsed.init(corrupted), -1) << "bit " << bit;
    EXPECT_FALSE(parsed.initialized) << "bit " << bit;
    EXPECT_STREQ(parsed.lastError(), "checksum") << "bit " << bit;
  }
}

// "C,123,0.349999994,1.5,...,1,0,0,1,6BB9": positions in the id, a float digit,
// a separating comma, an integer field and the last payload byte. Position 0
// (the 'C') is left out, a different letter is not a config line at all
INSTANTIATE_TEST_SUITE_P(Positions, CrcBitFlip,
                         ::testing::Values(2u, 8u, 17u, 82u, 100u));

} // namespace

///////////////////////////////////////////////////////////////////////////////
// number parsers of the firmware, through msgs::Config::init()
///////////////////////////////////////////////////////////////////////////////

namespace {

// fields of a config line: C,<id>,<8 floats>,<3 uint16>,<4 bools>,<crc>
constexpr size_t k_field_handshake_id = 1;       // parseUInt32_
constexpr size_t k_field_watchdog = 10;          // parseUInt16_
constexpr size_t k_field_invert_motor_left = 13; // parseBool_

struct ParserCase {
  const char *name;
  size_t field;
  const char *text;
  // the error init() reports, nullptr if the line is accepted
  const char *error;
  // the parsed value if accepted
  uint32_t value;
};

void PrintTo(const ParserCase &c, std::ostream *os) { *os << c.name; }

const ParserCase k_parser_cases[] = {
    // parseUInt32_ (handshake id)
    {"id_one", k_field_handshake_id, "1", nullptr, 1},
    {"id_uint32_max", k_field_handshake_id, "4294967295", nullptr, 4294967295u},
    {"id_leading_zeros", k_field_handshake_id, "007", nullptr, 7},
    // strtoull accepts a '+', the pi never sends one
    {"id_plus_sign", k_field_handshake_id, "+7", nullptr, 7},
    // intended: 0 means "no id parsed yet", the pi skips it
    {"id_zero", k_field_handshake_id, "0", "malformed", 0},
    {"id_zeros", k_field_handshake_id, "000", "malformed", 0},
    {"id_minus_sign", k_field_handshake_id, "-1", "malformed", 0},
    {"id_above_uint32_max", k_field_handshake_id, "4294967296", "malformed", 0},
    {"id_above_uint64_max", k_field_handshake_id, "18446744073709551616",
     "malformed", 0},
    {"id_empty", k_field_handshake_id, "", "malformed", 0},
    {"id_trailing_garbage", k_field_handshake_id, "7x", "malformed", 0},

    // parseUInt16_ (watchdog_timeout_ms). A value that parses but is out of
    // range is named by the range check instead of "malformed"
    {"uint16_value", k_field_watchdog, "200", nullptr, 200},
    {"uint16_leading_zeros", k_field_watchdog, "0200", nullptr, 200},
    {"uint16_plus_sign", k_field_watchdog, "+200", nullptr, 200},
    {"uint16_minus_zero", k_field_watchdog, "-0", "watchdog_timeout_ms", 0},
    {"uint16_minus_sign", k_field_watchdog, "-200", "malformed", 0},
    {"uint16_max", k_field_watchdog, "65535", "watchdog_timeout_ms", 0},
    // would wrap to 0 / 1 if narrowed before the range check
    {"uint16_overflow", k_field_watchdog, "65536", "malformed", 0},
    {"uint16_overflow_wraps_to_one", k_field_watchdog, "65537", "malformed", 0},
    {"uint16_overflow_uint64", k_field_watchdog, "18446744073709551616",
     "malformed", 0},
    {"uint16_empty", k_field_watchdog, "", "malformed", 0},
    {"uint16_float", k_field_watchdog, "200.0", "malformed", 0},
    {"uint16_exponent", k_field_watchdog, "2e2", "malformed", 0},

    // parseBool_ (invert_motor_left)
    {"bool_zero", k_field_invert_motor_left, "0", nullptr, 0},
    {"bool_one", k_field_invert_motor_left, "1", nullptr, 1},
    {"bool_leading_zero", k_field_invert_motor_left, "01", nullptr, 1},
    {"bool_plus_sign", k_field_invert_motor_left, "+1", nullptr, 1},
    {"bool_two", k_field_invert_motor_left, "2", "malformed", 0},
    {"bool_minus_one", k_field_invert_motor_left, "-1", "malformed", 0},
    // 65537 would be 1 after a narrowing to uint16_t
    {"bool_overflow", k_field_invert_motor_left, "65537", "malformed", 0},
    {"bool_word", k_field_invert_motor_left, "true", "malformed", 0},
    {"bool_empty", k_field_invert_motor_left, "", "malformed", 0},
};

class FirmwareNumberParser : public ::testing::TestWithParam<ParserCase> {};

TEST_P(FirmwareNumberParser, ParsesField) {
  const ParserCase &c = GetParam();
  std::vector<std::string> fields =
      splitFields(stripNewline(serializeConfig(validPayload(), 9)));
  fields.pop_back(); // the crc, recomputed below
  ASSERT_EQ(fields.size(), 17u);
  fields[c.field] = c.text;
  const std::string line = sign(joinFields(fields));

  msgs::Config parsed;
  const int result = parsed.init(line);
  if (c.error != nullptr) {
    EXPECT_EQ(result, -1) << line;
    EXPECT_STREQ(parsed.lastError(), c.error) << line;
    return;
  }

  ASSERT_EQ(result, 0) << line << ": " << parsed.lastError();
  switch (c.field) {
  case k_field_handshake_id:
    EXPECT_EQ(parsed.handshake_id, c.value);
    break;
  case k_field_watchdog:
    EXPECT_EQ(parsed.watchdog_timeout_ms, c.value);
    break;
  case k_field_invert_motor_left:
    EXPECT_EQ(parsed.invert_motor_left, c.value == 1);
    break;
  default:
    FAIL() << "no getter for field " << c.field;
  }
}

INSTANTIATE_TEST_SUITE_P(Fields, FirmwareNumberParser,
                         ::testing::ValuesIn(k_parser_cases),
                         [](const ::testing::TestParamInfo<ParserCase> &info) {
                           return std::string(info.param.name);
                         });

} // namespace
