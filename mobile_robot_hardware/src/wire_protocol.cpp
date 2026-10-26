#include "mobile_robot_hardware/wire_protocol.hpp"

// shared with the firmware, the include path points at esp32_firmware/
#include "inc/Crc16.hpp"

#include <cstdio>
#include <limits>
#include <stdexcept>

namespace mobile_robot_hardware {

namespace {

// std::to_string(float) prints 6 fixed decimals, which turns e.g. 1e-7 into
// "0.000000" and 0.99999994 into "1.000000" - values that pass
// validateConfig() on the pi but not on the esp32. 9 significant digits
// round-trip every float exactly, so both ends validate the same value
std::string formatFloat(float value) {
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(value));
  return buf;
}

} // namespace

std::string serializeConfig(const protocol::ConfigPayload &payload,
                            uint32_t handshakeId) {
  // the invert_* flags must go out as 0/1, not as true/false
  const std::string body =
      "C," + std::to_string(handshakeId) + "," + formatFloat(payload.kp) + "," +
      formatFloat(payload.ki) + "," + formatFloat(payload.integral_limit) +
      "," + formatFloat(payload.slope_left) + "," +
      formatFloat(payload.slope_right) + "," +
      formatFloat(payload.min_duty_left) + "," +
      formatFloat(payload.min_duty_right) + "," +
      formatFloat(payload.ticks_per_wheel_rev) + "," +
      std::to_string(payload.watchdog_timeout_ms) + "," +
      std::to_string(payload.control_rate_hz) + "," +
      std::to_string(payload.report_rate_hz) + "," +
      std::string(payload.invert_motor_left ? "1" : "0") + "," +
      std::string(payload.invert_motor_right ? "1" : "0") + "," +
      std::string(payload.invert_encoder_left ? "1" : "0") + "," +
      std::string(payload.invert_encoder_right ? "1" : "0");

  // lets the esp32 tell a corrupted line from the one the pi validated
  char crc[8];
  std::snprintf(crc, sizeof(crc), "%04X",
                protocol::crc16(body.c_str(), body.size()));
  return body + "," + crc + "\n";
}

std::string serializeMotorCmd(float leftVelCmd, float rightVelCmd) {
  return "M," + formatFloat(leftVelCmd) + "," + formatFloat(rightVelCmd) + "\n";
}

void parseEncoderReport(const std::string &line, int32_t &leftTicks,
                        int32_t &rightTicks, uint32_t &timestampUs) {
  const size_t p1 = line.find(',');
  const size_t p2 =
      p1 == std::string::npos ? std::string::npos : line.find(',', p1 + 1);
  const size_t p3 =
      p2 == std::string::npos ? std::string::npos : line.find(',', p2 + 1);
  if (p1 == std::string::npos || p2 == std::string::npos ||
      p3 == std::string::npos || line.substr(0, p1) != "E") {
    throw std::runtime_error("malformed encoder report: '" + line + "'");
  }

  try {
    leftTicks = std::stol(line.substr(p1 + 1, p2 - p1 - 1));
    rightTicks = std::stol(line.substr(p2 + 1, p3 - p2 - 1));
    timestampUs = static_cast<uint32_t>(std::stoul(line.substr(p3 + 1)));
  } catch (const std::exception &e) {
    throw std::runtime_error("malformed encoder report: '" + line +
                             "': " + e.what());
  }
}

bool parseHandshakeReply(const std::string &line, const std::string &prefix,
                         uint32_t &id, std::string &rest) {
  if (line.rfind(prefix, 0) != 0) {
    return false;
  }
  const size_t comma = line.find(',', prefix.size());
  const std::string idStr = line.substr(
      prefix.size(),
      comma == std::string::npos ? std::string::npos : comma - prefix.size());
  // std::stoul would accept a sign or leading whitespace and values that
  // alias a smaller id once cast to uint32_t
  if (idStr.empty()) {
    return false;
  }
  uint64_t value = 0;
  for (const char c : idStr) {
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10 + static_cast<uint64_t>(c - '0');
    if (value > std::numeric_limits<uint32_t>::max()) {
      return false;
    }
  }
  id = static_cast<uint32_t>(value);
  rest = comma == std::string::npos ? "" : line.substr(comma + 1);
  return true;
}

} // namespace mobile_robot_hardware
