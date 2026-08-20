#include "mobile_robot_hardware/mobile_robot_hardware.hpp"

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <algorithm>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>

using namespace mobile_robot_hardware;
using namespace hardware_interface;

namespace {

bool hasInterface(const std::vector<InterfaceInfo> &interfaces,
                  const std::string &name) {
  return std::any_of(
      interfaces.begin(), interfaces.end(),
      [&name](const InterfaceInfo &info) { return info.name == name; });
}

// check that the ros2_control URDF declares the joints and interfaces
// read()/write() assume
// @throws std::runtime_error describing the first mismatch found
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

struct ParsedParams {
  std::string serialPort;
  uint32_t baudRate = 0;
  protocol::ConfigPayload config;
};

// look up a required <param> of the <ros2_control> block and convert it
// @throws std::runtime_error naming the parameter, if it is missing, malformed
//         or out of range for T
template <typename T>
T parseParam(const std::unordered_map<std::string, std::string> &params,
             const std::string &name) {
  const auto it = params.find(name);
  if (it == params.end()) {
    throw std::runtime_error("missing required hardware parameter '" + name +
                             "'");
  }

  try {
    if constexpr (std::is_same_v<T, std::string>) {
      return it->second;
    } else if constexpr (std::is_same_v<T, bool>) {
      return parse_bool(it->second);
    } else {
      const double value = hardware_interface::stod(it->second);
      // check range for integral types (don't check for floats)
      if constexpr (std::is_integral_v<T>) {
        if (value < static_cast<double>(std::numeric_limits<T>::min()) ||
            value > static_cast<double>(std::numeric_limits<T>::max())) {
          throw std::out_of_range("out of range for its type");
        }
      }
      return static_cast<T>(value);
    }
  } catch (const std::exception &e) {
    throw std::runtime_error("hardware parameter '" + name +
                             "' has invalid value '" + it->second +
                             "': " + e.what());
  }
}

// read the <param> tags of the <ros2_control> block. The urdf names carry a
// pid_/ff_ prefix for readability, the payload fields are named after the wire
// format instead
// @throws std::runtime_error naming the offending parameter
ParsedParams
parseParams(const std::unordered_map<std::string, std::string> &p) {
  ParsedParams parsed;

  parsed.serialPort = parseParam<std::string>(p, "serial_port");
  parsed.baudRate = parseParam<uint32_t>(p, "baud_rate");
  if (parsed.baudRate == 0) {
    throw std::runtime_error("hardware parameter 'baud_rate' must be > 0");
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

  parsed.config.invert_left = parseParam<bool>(p, "invert_left");
  parsed.config.invert_right = parseParam<bool>(p, "invert_right");

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

} // namespace

CallbackReturn
MobileRobotHardware::on_init(const HardwareComponentInterfaceParams &params) {
  // call base class implementation to parse <ros2_control> URDF tag
  auto ret = SystemInterface::on_init(params);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  try {
    validateJoints(info_, expected_joints_);

    ParsedParams parsed = parseParams(info_.hardware_parameters);
    serialPort_ = std::move(parsed.serialPort);
    baudRate_ = parsed.baudRate;
    config_ = parsed.config;
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->get_logger(), "Configured for esp32 on %s at %u baud",
              serialPort_.c_str(), baudRate_);
  return CallbackReturn::SUCCESS;
}

// CallbackReturn MobileRobotHardware::on_configure(
//     const rclcpp_lifecycle::State &previous_state) {
//
//   return CallbackReturn::SUCCESS;
// }
//
// CallbackReturn
// MobileRobotHardware::on_cleanup(const rclcpp_lifecycle::State
// &previous_state) {
//
//   return CallbackReturn::SUCCESS;
// }
