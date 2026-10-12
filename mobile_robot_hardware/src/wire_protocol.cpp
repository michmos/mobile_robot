#include "mobile_robot_hardware/wire_protocol.hpp"

#include <limits>

namespace mobile_robot_hardware {

std::string serializeConfig(const protocol::ConfigPayload &payload,
                            uint32_t handshakeId) {
  // invert_left/invert_right must go out as 0/1, not as true/false
  return "C," + std::to_string(handshakeId) + "," +
         std::to_string(payload.kp) + "," + std::to_string(payload.ki) + "," +
         std::to_string(payload.integral_limit) + "," +
         std::to_string(payload.slope_left) + "," +
         std::to_string(payload.slope_right) + "," +
         std::to_string(payload.min_duty_left) + "," +
         std::to_string(payload.min_duty_right) + "," +
         std::to_string(payload.ticks_per_wheel_rev) + "," +
         std::to_string(payload.watchdog_timeout_ms) + "," +
         std::to_string(payload.control_rate_hz) + "," +
         std::to_string(payload.report_rate_hz) + "," +
         std::string(payload.invert_left ? "1" : "0") + "," +
         std::string(payload.invert_right ? "1" : "0") + "\n";
}

bool parseHandshakeReply(const std::string &line, const std::string &prefix,
                         uint32_t &id, std::string &rest) {
  if (line.rfind(prefix, 0) != 0) {
    return false;
  }
  const size_t comma = line.find(',', prefix.size());
  const std::string idStr =
      line.substr(prefix.size(), comma == std::string::npos
                                     ? std::string::npos
                                     : comma - prefix.size());
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
