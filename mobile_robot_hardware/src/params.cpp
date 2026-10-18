#include "mobile_robot_hardware/params.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>

using namespace hardware_interface;

namespace mobile_robot_hardware::detail {

namespace {

bool hasInterface(const std::vector<InterfaceInfo> &interfaces,
                  const std::string &name) {
  return std::any_of(
      interfaces.begin(), interfaces.end(),
      [&name](const InterfaceInfo &info) { return info.name == name; });
}

} // namespace

void validateJoints(const std::vector<ComponentInfo> &joints,
                    const std::vector<std::string> &expectedJoints) {
  // check joint amount
  if (joints.size() != std::size(expectedJoints)) {
    throw std::runtime_error(
        "Expected " + std::to_string(expectedJoints.size()) +
        " joints, but found " + std::to_string(joints.size()) +
        " in the ros2_control URDF");
  }

  for (auto expected : expectedJoints) {
    // check joint names
    const auto joint = std::find_if(
        joints.begin(), joints.end(),
        [expected](const ComponentInfo &j) { return j.name == expected; });
    if (joint == joints.end()) {
      throw std::runtime_error("Joint '" + expected +
                               "' is missing from the ros2_control URDF");
    }

    // check joint interfaces
    if (joint->command_interfaces.size() != 1 ||
        joint->command_interfaces[0].name != HW_IF_VELOCITY) {
      throw std::runtime_error("Joint '" + expected +
                               "' must have exactly one '" + HW_IF_VELOCITY +
                               "' command interface");
    }
    if (joint->state_interfaces.size() != 2 ||
        !hasInterface(joint->state_interfaces, HW_IF_POSITION) ||
        !hasInterface(joint->state_interfaces, HW_IF_VELOCITY)) {
      throw std::runtime_error(
          "Joint '" + expected + "' must have exactly a '" + HW_IF_POSITION +
          "' and a '" + HW_IF_VELOCITY + "' state interface");
    }
  }
}

ParsedParams
parseParams(const std::unordered_map<std::string, std::string> &p) {
  ParsedParams parsed;

  parsed.serialPort = parseParam<std::string>(p, "serial_port");
  parsed.baudRate = parseParam<uint32_t>(p, "baud_rate");
  if (parsed.baudRate == 0) {
    throw std::runtime_error("hardware parameter 'baud_rate' must be > 0");
  }

  // optional: the default covers the esp32 booting, the tests shorten it so a
  // failing handshake doesn't take seconds
  if (p.count("handshake_timeout_ms") != 0) {
    const uint32_t timeoutMs = parseParam<uint32_t>(p, "handshake_timeout_ms");
    if (timeoutMs == 0) {
      throw std::runtime_error(
          "hardware parameter 'handshake_timeout_ms' must be > 0");
    }
    parsed.handshakeTimeout = std::chrono::milliseconds(timeoutMs);
  }

  parsed.config.kp = parseParam<float>(p, "pid_kp");
  parsed.config.ki = parseParam<float>(p, "pid_ki");
  parsed.config.integral_limit = parseParam<float>(p, "pid_integral_limit");

  parsed.config.slope_left = parseParam<float>(p, "ff_slope_left");
  parsed.config.slope_right = parseParam<float>(p, "ff_slope_right");
  parsed.config.min_duty_left = parseParam<float>(p, "ff_min_duty_left");
  parsed.config.min_duty_right = parseParam<float>(p, "ff_min_duty_right");

  parsed.config.ticks_per_wheel_rev =
      parseParam<float>(p, "ticks_per_wheel_rev");

  parsed.config.watchdog_timeout_ms =
      parseParam<uint16_t>(p, "watchdog_timeout_ms");
  parsed.config.control_rate_hz = parseParam<uint16_t>(p, "control_rate_hz");
  parsed.config.report_rate_hz = parseParam<uint16_t>(p, "report_rate_hz");

  parsed.config.invert_motor_left = parseParam<bool>(p, "invert_motor_left");
  parsed.config.invert_motor_right = parseParam<bool>(p, "invert_motor_right");
  parsed.config.invert_encoder_left =
      parseParam<bool>(p, "invert_encoder_left");
  parsed.config.invert_encoder_right =
      parseParam<bool>(p, "invert_encoder_right");

  // the same check the firmware runs on the received Config message, so a bad
  // value fails at load time rather than as a NACK once the port is open
  const protocol::e_config_field ret = protocol::validateConfig(parsed.config);
  if (ret != protocol::CONFIG_OK) {
    throw std::runtime_error(
        "hardware parameter for '" +
        std::string(protocol::configFieldName(ret)) +
        "' is out of range, the esp32 would reject this configuration");
  }

  return parsed;
}

double ticksToRad(int32_t ticks, float ticksPerWheelRev) {
  return (ticks * 2.0 * M_PI) / ticksPerWheelRev;
}

} // namespace mobile_robot_hardware::detail
