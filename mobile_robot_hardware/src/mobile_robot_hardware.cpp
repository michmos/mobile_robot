#include "mobile_robot_hardware/mobile_robot_hardware.hpp"

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
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

// without a timeout, a dead esp32 would hang the lifecycle transition forever
// instead of failing it
constexpr std::chrono::milliseconds k_setup_timeout{5000};
constexpr std::chrono::milliseconds k_ack_timeout{2000};

// M,<left_vel_cmd>,<right_vel_cmd>\n
std::string serializeMotorCmd(float leftVelCmd, float rightVelCmd) {
  return "M," + std::to_string(leftVelCmd) + "," + std::to_string(rightVelCmd) +
         "\n";
}
// E,<left_ticks>,<right_ticks>,<us>\n - see esp32_firmware/inc/Messages.hpp
// EncoderData; line has already had its trailing '\n' stripped by readLine_
// @throws std::runtime_error if the line is malformed
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

double ticksToRad(int32_t ticks, float ticksPerWheelRev) {
  return (ticks * 2.0 * M_PI) / ticksPerWheelRev;
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
    validateJoints(info_.joints, expected_joints_);

    ParsedParams parsed = parseParams(info_.hardware_parameters);
    serialPort_ = std::move(parsed.serialPort);
    baudRate_ = parsed.baudRate;
    config_.set(parsed.config);
  } catch (const std::runtime_error &e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    return CallbackReturn::ERROR;
  }

  declareTunableParams_();

  RCLCPP_INFO(this->get_logger(), "Configured for esp32 on %s at %u baud",
              serialPort_.c_str(), baudRate_);
  return CallbackReturn::SUCCESS;
}

protocol::ConfigPayload MobileRobotHardware::Config::get() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return payload_;
}

void MobileRobotHardware::Config::set(
    const protocol::ConfigPayload &newConfig) {
  std::lock_guard<std::mutex> lock(mutex_);
  payload_ = newConfig;
}

std::string MobileRobotHardware::Config::serialize() const {
  std::lock_guard<std::mutex> lock(mutex_);
  // invert_left/invert_right must go out as 0/1, not as true/false
  return "C," + std::to_string(payload_.kp) + "," +
         std::to_string(payload_.ki) + "," +
         std::to_string(payload_.integral_limit) + "," +
         std::to_string(payload_.slope_left) + "," +
         std::to_string(payload_.slope_right) + "," +
         std::to_string(payload_.min_duty_left) + "," +
         std::to_string(payload_.min_duty_right) + "," +
         std::to_string(payload_.ticks_per_wheel_rev) + "," +
         std::to_string(payload_.watchdog_timeout_ms) + "," +
         std::to_string(payload_.control_rate_hz) + "," +
         std::to_string(payload_.report_rate_hz) + "," +
         std::string(payload_.invert_left ? "1" : "0") + "," +
         std::string(payload_.invert_right ? "1" : "0") + "\n";
}

void MobileRobotHardware::declareTunableParams_() {
  auto node = get_node();
  const protocol::ConfigPayload initial = config_.get();
  node->declare_parameter("pid_kp", static_cast<double>(initial.kp));
  node->declare_parameter("pid_ki", static_cast<double>(initial.ki));
  node->declare_parameter("pid_integral_limit",
                          static_cast<double>(initial.integral_limit));
  node->declare_parameter("ff_slope_left",
                          static_cast<double>(initial.slope_left));
  node->declare_parameter("ff_slope_right",
                          static_cast<double>(initial.slope_right));
  node->declare_parameter("ff_min_duty_left",
                          static_cast<double>(initial.min_duty_left));
  node->declare_parameter("ff_min_duty_right",
                          static_cast<double>(initial.min_duty_right));

  paramCallbackHandle_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        return onParamsSet_(parameters);
      });
}

rcl_interfaces::msg::SetParametersResult MobileRobotHardware::onParamsSet_(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  protocol::ConfigPayload candidate = config_.get();

  try {
    for (const auto &param : parameters) {
      const auto &name = param.get_name();
      const float value = static_cast<float>(param.as_double());
      if (name == "pid_kp") {
        candidate.kp = value;
      } else if (name == "pid_ki") {
        candidate.ki = value;
      } else if (name == "pid_integral_limit") {
        candidate.integral_limit = value;
      } else if (name == "ff_slope_left") {
        candidate.slope_left = value;
      } else if (name == "ff_slope_right") {
        candidate.slope_right = value;
      } else if (name == "ff_min_duty_left") {
        candidate.min_duty_left = value;
      } else if (name == "ff_min_duty_right") {
        candidate.min_duty_right = value;
      }
      // anything else isn't one of ours (shouldn't happen, we only declare
      // the names above), leave candidate as-is
    }
  } catch (const std::exception &e) {
    result.successful = false;
    result.reason = std::string("invalid parameter value: ") + e.what();
    return result;
  }

  const protocol::e_config_field ret = protocol::validateConfig(candidate);
  if (ret != protocol::CONFIG_OK) {
    result.successful = false;
    result.reason = "value for '" +
                    std::string(protocol::configFieldName(ret)) +
                    "' is out of range, the esp32 would reject it";
    return result;
  }

  config_.set(candidate);
  const std::string serialized = config_.serialize();

  // best-effort: not yet connected (or mid-reconnect) just means the new
  // config_ takes effect on the next handshake instead of immediately
  if (serial_.port && serial_.port->is_open()) {
    try {
      serial_.port->send(
          std::vector<uint8_t>(serialized.begin(), serialized.end()));
    } catch (const std::exception &e) {
      result.successful = false;
      result.reason =
          std::string("failed to send configuration to esp32: ") + e.what();
      return result;
    }
  }

  return result;
}

bool MobileRobotHardware::readLine_(std::string &line,
                                    std::chrono::milliseconds timeout) {
  std::unique_lock<std::mutex> lock(serial_.rxMutex);
  const auto hasLine = [this] {
    return serial_.rxBuffer.find('\n') != std::string::npos;
  };
  if (!serial_.rxCv.wait_for(lock, timeout, hasLine)) {
    return false;
  }

  const size_t nl = serial_.rxBuffer.find('\n');
  line = serial_.rxBuffer.substr(0, nl);
  serial_.rxBuffer.erase(0, nl + 1);
  return true;
}

template <typename Predicate>
std::optional<std::string>
MobileRobotHardware::readLineUntil_(std::chrono::milliseconds timeout,
                                    Predicate matches) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string line;
  while (true) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
    if (remaining <= std::chrono::milliseconds(0) ||
        !readLine_(line, remaining)) {
      return {};
    }
    if (matches(line)) {
      return line;
    }
  }
}

void MobileRobotHardware::performHandshake_(
    std::chrono::milliseconds setupTimeout,
    std::chrono::milliseconds ackTimeout) {
  // check for setup event
  {
    auto ret = readLineUntil_(setupTimeout, [](const std::string &line) {
      return line == "S,SETUP";
    });
    if (!ret.has_value()) {
      RCLCPP_WARN(this->get_logger(), "No SETUP message received from esp32");
    }
  }

  // send config
  const std::string configLine = config_.serialize();
  serial_.port->send(
      std::vector<uint8_t>(configLine.begin(), configLine.end()));

  // wait for acknowledgment
  auto ret = readLineUntil_(ackTimeout, [](const std::string &line) {
    return line == "S,ACK" || line.rfind("S,NACK,", 0) == 0;
  });
  if (!ret.has_value()) {
    throw std::runtime_error(
        "timed out waiting for the esp32 to acknowledge the configuration");
  }
  if (ret.value().rfind("S,NACK,", 0) == 0) {
    throw std::runtime_error("esp32 rejected configuration field '" +
                             ret.value().substr(7) + "'");
  }
  RCLCPP_INFO(this->get_logger(), "esp32 on %s acknowledged configuration",
              serialPort_.c_str());
}

CallbackReturn
MobileRobotHardware::on_configure(const rclcpp_lifecycle::State &) {
  using namespace drivers::serial_driver;

  // 8N1, no flow control matches the esp32's default UART setup
  const SerialPortConfig serialConfig(baudRate_, FlowControl::NONE,
                                      Parity::NONE, StopBits::ONE);

  try {
    // validate and cache handles for later use
    leftPositionState_ =
        get_state_interface_handle(expected_joints_[0] + "/" + HW_IF_POSITION);
    leftVelocityState_ =
        get_state_interface_handle(expected_joints_[0] + "/" + HW_IF_VELOCITY);
    rightPositionState_ =
        get_state_interface_handle(expected_joints_[1] + "/" + HW_IF_POSITION);
    rightVelocityState_ =
        get_state_interface_handle(expected_joints_[1] + "/" + HW_IF_VELOCITY);

    leftVelocityCommand_ = get_command_interface_handle(expected_joints_[0] +
                                                        "/" + HW_IF_VELOCITY);
    rightVelocityCommand_ = get_command_interface_handle(expected_joints_[1] +
                                                         "/" + HW_IF_VELOCITY);

    // setup serial port
    serial_.driver.init_port(serialPort_, serialConfig);
    serial_.port = serial_.driver.port();
    serial_.port->open();
    // // re-arms itself after every callback, so this stays registered for as
    // // long as the port is open
    serial_.port->async_receive(
        [this](std::vector<uint8_t> &data, const size_t &length) {
          std::lock_guard<std::mutex> lock(serial_.rxMutex);
          serial_.rxBuffer.append(reinterpret_cast<const char *>(data.data()),
                                  length);
          serial_.rxCv.notify_all();
        });

    performHandshake_(k_setup_timeout, k_ack_timeout);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure esp32 on '%s': %s",
                 serialPort_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn
MobileRobotHardware::on_cleanup(const rclcpp_lifecycle::State &) {
  // close() can throw on an asio-level error; log it but still tear down our
  // own state below so a bad close doesn't leave stale handles/port around
  try {
    if (serial_.port && serial_.port->is_open()) {
      serial_.port->close();
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "Error closing serial port '%s': %s",
                serialPort_.c_str(), e.what());
  }
  serial_.port.reset();

  {
    std::lock_guard<std::mutex> lock(serial_.rxMutex);
    serial_.rxBuffer.clear();
  }

  leftPositionState_.reset();
  leftVelocityState_.reset();
  rightPositionState_.reset();
  rightVelocityState_.reset();
  leftVelocityCommand_.reset();
  rightVelocityCommand_.reset();

  RCLCPP_INFO(this->get_logger(), "Closed serial port %s", serialPort_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
MobileRobotHardware::on_activate(const rclcpp_lifecycle::State &) {
  // reset command interfaces to safe value
  if (!leftVelocityCommand_->set_value(0.0, true) ||
      !rightVelocityCommand_->set_value(0.0, true)) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to reset velocity command interfaces");
    return CallbackReturn::ERROR;
  }

  // clear buffer so the first velocity sample in read() isn't computed against
  // a stale timestamp
  {
    std::lock_guard<std::mutex> lock(serial_.rxMutex);
    serial_.rxBuffer.clear();
  }
  ticks_ = Ticks{};
  lastSampleTimeUs_ = 0;
  hasEncoderBaseline_ = false;

  RCLCPP_INFO(this->get_logger(), "Activated esp32 on %s", serialPort_.c_str());
  return CallbackReturn::SUCCESS;
}

void MobileRobotHardware::sendMotorCmd_(float leftVelCmd, float rightVelCmd) {
  const std::string cmd = serializeMotorCmd(leftVelCmd, rightVelCmd);
  serial_.port->send(std::vector<uint8_t>(cmd.begin(), cmd.end()));
}

void MobileRobotHardware::handleRestart_() {
  RCLCPP_WARN(this->get_logger(),
              "esp32 on %s sent a SETUP event while active, assuming it "
              "restarted; resending configuration",
              serialPort_.c_str());

  ticks_.leftLast = 0;
  ticks_.rightLast = 0;
  hasEncoderBaseline_ = false;

  try {
    const std::string configLine = config_.serialize();
    serial_.port->send(
        std::vector<uint8_t>(configLine.begin(), configLine.end()));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to resend configuration to esp32 on '%s': %s",
                 serialPort_.c_str(), e.what());
  }
}

CallbackReturn
MobileRobotHardware::on_deactivate(const rclcpp_lifecycle::State &) {
  try {
    sendMotorCmd_(0.0f, 0.0f);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to send stop command to esp32 on '%s': %s",
                 serialPort_.c_str(), e.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated esp32 on %s, motors stopped",
              serialPort_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MobileRobotHardware::on_shutdown(
    const rclcpp_lifecycle::State &previous_state) {
  // Best-effort: this is final, so log and keep going rather than bailing out
  // with an open port or unstopped motors
  if (previous_state.label() == lifecycle_state_names::ACTIVE &&
      on_deactivate(previous_state) != CallbackReturn::SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "Failed to stop motors during shutdown");
  }
  if ((previous_state.label() == lifecycle_state_names::ACTIVE ||
       previous_state.label() == lifecycle_state_names::INACTIVE) &&
      on_cleanup(previous_state) != CallbackReturn::SUCCESS) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to release resources during shutdown");
  }

  RCLCPP_INFO(this->get_logger(),
              "Shut down hardware interface for esp32 on %s",
              serialPort_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn
MobileRobotHardware::on_error(const rclcpp_lifecycle::State &previous_state) {
  if (previous_state.label() == lifecycle_state_names::ACTIVE &&
      on_deactivate(previous_state) != CallbackReturn::SUCCESS) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to stop motors during error recovery");
  }
  // unlike on_shutdown, run unconditionally: a failed on_configure() can
  // leave the port open despite previous_state being UNCONFIGURED
  if (on_cleanup(previous_state) != CallbackReturn::SUCCESS) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to release resources during error recovery");
  }

  RCLCPP_INFO(this->get_logger(),
              "Recovered from error, esp32 on %s is unconfigured",
              serialPort_.c_str());
  return CallbackReturn::SUCCESS;
}

void MobileRobotHardware::processNewLines_() {
  std::optional<std::string> latestEncoderLine;
  std::string line;
  // 0ms timeout makes readLine_ non-blocking, draining whatever is already
  // buffered instead of waiting for more to arrive
  while (readLine_(line, std::chrono::milliseconds(0))) {
    if (line.rfind("L,", 0) == 0) {
      RCLCPP_INFO(this->get_logger(), "esp32: %s", line.c_str() + 2);
    } else if (line == "S,SETUP") {
      handleRestart_();
      // encoder lines from before the restart belong to the old tick counter
      latestEncoderLine.reset();
    } else if (line == "S,ACK") {
      RCLCPP_INFO(this->get_logger(), "esp32 sent ACK");
    } else if (line.rfind("S,NACK,", 0) == 0) {
      RCLCPP_ERROR(this->get_logger(), "esp32 sent NACK, '%s'",
                   line.c_str() + 7);
    } else if (line.rfind("E,", 0) == 0) {
      latestEncoderLine = line;
    }
    // anything else is unrecognized and skipped
  }

  // only the newest encoder report matters, older ones are superseded by it
  if (latestEncoderLine.has_value()) {
    updateEncoderState_(*latestEncoderLine);
  }
}

void MobileRobotHardware::updateEncoderState_(const std::string &line) {
  auto warnIfFailed = [this](bool ok, const char *what) {
    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Could not update %s state interface",
                  what);
    }
  };

  int32_t leftTicks, rightTicks;
  uint32_t timestampUs;
  try {
    parseEncoderReport(line, leftTicks, rightTicks, timestampUs);
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->get_logger(), "%s", e.what());
    return;
  }

  const float ticksPerWheelRev = config_.get().ticks_per_wheel_rev;

  // update total ticks - delta required in cas of esp restart
  const int32_t leftDelta = leftTicks - ticks_.leftLast;
  const int32_t rightDelta = rightTicks - ticks_.rightLast;
  ticks_.leftTotal += leftDelta;
  ticks_.rightTotal += rightDelta;
  ticks_.leftLast = leftTicks;
  ticks_.rightLast = rightTicks;

  // set position
  warnIfFailed(leftPositionState_->set_value(
                   ticksToRad(ticks_.leftTotal, ticksPerWheelRev)),
               "left position");
  warnIfFailed(rightPositionState_->set_value(
                   ticksToRad(ticks_.rightTotal, ticksPerWheelRev)),
               "right position");

  // set velocity, averaged over the time since the previous processed sample
  if (hasEncoderBaseline_) {
    // uint32_t subtraction wraps correctly
    const double dt_s = (timestampUs - lastSampleTimeUs_) * 1e-6;
    if (dt_s > 0.0) {
      double leftVelocity = ticksToRad(leftDelta, ticksPerWheelRev) / dt_s;
      double rightVelocity = ticksToRad(rightDelta, ticksPerWheelRev) / dt_s;
      warnIfFailed(leftVelocityState_->set_value(leftVelocity),
                   "left velocity");
      warnIfFailed(rightVelocityState_->set_value(rightVelocity),
                   "right velocity");
    }
  }

  lastSampleTimeUs_ = timestampUs;
  hasEncoderBaseline_ = true;
}

return_type MobileRobotHardware::read(const rclcpp::Time &,
                                      const rclcpp::Duration &) {
  processNewLines_();
  return return_type::OK;
}

return_type MobileRobotHardware::write(const rclcpp::Time &,
                                       const rclcpp::Duration &) {
  // non-blocking: a realtime write() must not wait on a lock
  const auto leftCmd = leftVelocityCommand_->get_optional<double>();
  const auto rightCmd = rightVelocityCommand_->get_optional<double>();
  if (!leftCmd.has_value() || !rightCmd.has_value()) {
    RCLCPP_WARN(this->get_logger(),
                "Could not read velocity command interfaces, skipping write");
    return return_type::OK;
  }

  try {
    sendMotorCmd_(static_cast<float>(*leftCmd), static_cast<float>(*rightCmd));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to send motor command to esp32 on '%s': %s",
                 serialPort_.c_str(), e.what());
    return return_type::ERROR;
  }

  return return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(mobile_robot_hardware::MobileRobotHardware,
                       hardware_interface::SystemInterface)
