#ifndef MOBILE_ROBOT_HARDWARE_PARAMS_HPP
#define MOBILE_ROBOT_HARDWARE_PARAMS_HPP

// helpers of the hardware plugin that turn the <ros2_control> URDF block into
// its configuration. No I/O and no node, so they can be tested on their own

// shared with the firmware, the include path points at esp32_firmware/
#include "inc/ConfigPayload.hpp"

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace mobile_robot_hardware::detail {

// check that the ros2_control URDF declares the joints and interfaces
// read()/write() assume
// @throws std::runtime_error describing the first mismatch found
void validateJoints(const std::vector<hardware_interface::ComponentInfo> &joints,
                    const std::vector<std::string> &expectedJoints);

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
      return hardware_interface::parse_bool(it->second);
    } else {
      const double value = hardware_interface::stod(it->second);
      // stod() only guarantees a finite double, which can still be out of
      // range for T: a float would become inf, an integer is UB
      if (value < static_cast<double>(std::numeric_limits<T>::lowest()) ||
          value > static_cast<double>(std::numeric_limits<T>::max())) {
        throw std::out_of_range("out of range for its type");
      }
      // the cast would truncate silently, e.g. 200.5 Hz to 200 Hz
      if constexpr (std::is_integral_v<T>) {
        if (std::trunc(value) != value) {
          throw std::invalid_argument("not an integer");
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
parseParams(const std::unordered_map<std::string, std::string> &p);

double ticksToRad(int32_t ticks, float ticksPerWheelRev);

} // namespace mobile_robot_hardware::detail

#endif
