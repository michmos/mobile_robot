#ifndef MOBILE_ROBOT_HARDWARE_TEST_HARDWARE_FIXTURE_HPP
#define MOBILE_ROBOT_HARDWARE_TEST_HARDWARE_FIXTURE_HPP

// gtest fixture that drives the hardware plugin the way the resource manager
// does, against a fake esp32 on a pty (see fake_esp32.hpp). Shared by the
// component tests of the plugin

#include "fake_esp32.hpp"
#include "mobile_robot_hardware/mobile_robot_hardware.hpp"

// the firmware's parser, compiled against test/arduino_stub/Arduino.h
#include "inc/Messages.hpp"

#include <gtest/gtest.h>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/lifecycle_state_names.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace mobile_robot_hardware::test {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using mobile_robot_hardware::MobileRobotHardware;
using namespace std::chrono_literals;

using Params = std::unordered_map<std::string, std::string>;

// the <param> tags of mobile_robot_nodes/description/ros2_control.xacro,
// keep in sync with it. serial_port is replaced by the pty in most tests
inline Params xacroDefaults() {
  return {
      {"serial_port", "/dev/esp32"},
      {"baud_rate", "115200"},
      {"pid_kp", "0.5"},
      {"pid_ki", "2.0"},
      {"pid_integral_limit", "0.3"},
      {"ff_slope_left", "0.1"},
      {"ff_slope_right", "0.1"},
      {"ff_min_duty_left", "0.0"},
      {"ff_min_duty_right", "0.0"},
      {"ticks_per_wheel_rev", "480"},
      {"watchdog_timeout_ms", "200"},
      {"control_rate_hz", "200"},
      {"report_rate_hz", "50"},
      {"invert_left", "true"},
      {"invert_right", "false"},
  };
}

// long enough for any handshake against an acking fake, short enough that a
// broken test fails instead of hanging for the 10 s default
constexpr auto k_test_handshake_timeout = "5000";
// for the tests that wait for a handshake to fail
constexpr auto k_short_handshake_timeout = "200";

// bound for waiting on something that should happen right away
constexpr auto k_wait = 2s;

inline hardware_interface::InterfaceInfo makeInterface(const std::string &name) {
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.data_type = "double";
  info.size = 1;
  info.enable_limits = false;
  return info;
}

inline hardware_interface::ComponentInfo
makeJoint(const std::string &name,
          const std::vector<std::string> &commandInterfaces = {"velocity"},
          const std::vector<std::string> &stateInterfaces = {"velocity",
                                                             "position"}) {
  hardware_interface::ComponentInfo joint{};
  joint.name = name;
  joint.type = "joint";
  for (const auto &ci : commandInterfaces) {
    joint.command_interfaces.push_back(makeInterface(ci));
  }
  for (const auto &si : stateInterfaces) {
    joint.state_interfaces.push_back(makeInterface(si));
  }
  return joint;
}

// the joints of ros2_control.xacro
inline std::vector<hardware_interface::ComponentInfo> xacroJoints() {
  return {makeJoint("left_wheel_joint"), makeJoint("right_wheel_joint")};
}

// what the resource manager parses from the <ros2_control> block. Built by
// hand instead of from a URDF string, the parser would also need the links
// and joints of the whole robot
inline hardware_interface::HardwareInfo
makeInfo(const Params &params,
         const std::vector<hardware_interface::ComponentInfo> &joints) {
  hardware_interface::HardwareInfo info{};
  info.name = "mobile_robot_ros2_control";
  info.type = "system";
  info.hardware_plugin_name = "mobile_robot_hardware/MobileRobotHardware";
  info.is_async = false;
  info.hardware_parameters = params;
  info.joints = joints;
  return info;
}

inline rclcpp_lifecycle::State state(uint8_t id, const char *label) {
  return rclcpp_lifecycle::State(id, label);
}
inline const rclcpp_lifecycle::State k_unconfigured =
    state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
          hardware_interface::lifecycle_state_names::UNCONFIGURED);
inline const rclcpp_lifecycle::State k_inactive =
    state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
          hardware_interface::lifecycle_state_names::INACTIVE);
inline const rclcpp_lifecycle::State k_active =
    state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
          hardware_interface::lifecycle_state_names::ACTIVE);

struct MotorCmd {
  float left;
  float right;
};

// "M,<left>,<right>"
inline std::optional<MotorCmd> parseMotorCmd(const std::string &line) {
  if (line.rfind("M,", 0) != 0) {
    return std::nullopt;
  }
  char *end = nullptr;
  MotorCmd cmd{};
  cmd.left = std::strtof(line.c_str() + 2, &end);
  if (*end != ',') {
    return std::nullopt;
  }
  cmd.right = std::strtof(end + 1, &end);
  if (*end != '\0') {
    return std::nullopt;
  }
  return cmd;
}


class MobileRobotHardwareTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    fake_ = std::make_unique<FakeEsp32>();
  }

  void TearDown() override {
    // the plugin first, it holds the port the fake plays on
    commands_.clear();
    states_.clear();
    hw_.reset();
    fake_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  // runs what the resource manager does on loading a component: init() and
  // exporting the interfaces, which on_configure() looks up by name
  CallbackReturn
  init(const Params &params,
       const std::vector<hardware_interface::ComponentInfo> &joints =
           xacroJoints()) {
    hw_ = std::make_unique<MobileRobotHardware>();
    hardware_interface::HardwareComponentParams componentParams;
    componentParams.hardware_info = makeInfo(params, joints);
    componentParams.logger = rclcpp::get_logger("test_mobile_robot_hardware");
    componentParams.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    // without an executor no node is created, and with it no parameters
    componentParams.executor = executor_;
    const CallbackReturn ret = hw_->init(componentParams);
    if (ret == CallbackReturn::SUCCESS) {
      states_ = hw_->on_export_state_interfaces();
      commands_ = hw_->on_export_command_interfaces();
    }
    return ret;
  }

  // the xacro defaults on the fake's port, with `overrides` applied
  Params fakeParams(const Params &overrides = {}) const {
    Params params = xacroDefaults();
    params["serial_port"] = fake_->slavePath();
    params["handshake_timeout_ms"] = k_test_handshake_timeout;
    for (const auto &[name, value] : overrides) {
      params[name] = value;
    }
    return params;
  }

  void configureAndActivate() {
    ASSERT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::SUCCESS);
    ASSERT_EQ(hw_->on_activate(k_inactive), CallbackReturn::SUCCESS);
  }

  void setCommand(double left, double right) {
    ASSERT_EQ(commands_.size(), 2u);
    for (const auto &command : commands_) {
      const bool isLeft = command->get_name() == "left_wheel_joint/velocity";
      ASSERT_TRUE(command->set_value(isLeft ? left : right, true));
    }
  }

  // write() and return the motor command the fake received for it
  std::optional<MotorCmd> writeAndReceive() {
    EXPECT_EQ(hw_->write(now_, period_), return_type::OK);
    const auto line = fake_->waitForLine("M,", k_wait);
    if (!line) {
      return std::nullopt;
    }
    return parseMotorCmd(*line);
  }

  // calls read() until the fake received a line starting with `prefix`.
  // read() does not block, what it reacts to arrives asynchronously
  std::optional<std::string>
  readUntilFakeReceives(const std::string &prefix,
                        std::chrono::milliseconds timeout = k_wait) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      EXPECT_EQ(hw_->read(now_, period_), return_type::OK);
      if (auto line = fake_->waitForLine(prefix, 1ms)) {
        return line;
      }
    }
    return std::nullopt;
  }

  // read() + write() until the plugin sends a non-zero motor command, which it
  // only does once the running handshake is acked
  // @return: the first non-zero command, nullopt on timeout
  std::optional<MotorCmd> cycleUntilNonZeroCmd() {
    const auto deadline = std::chrono::steady_clock::now() + k_wait;
    while (std::chrono::steady_clock::now() < deadline) {
      EXPECT_EQ(hw_->read(now_, period_), return_type::OK);
      const auto cmd = writeAndReceive();
      if (!cmd) {
        return std::nullopt;
      }
      if (cmd->left != 0.0f || cmd->right != 0.0f) {
        return cmd;
      }
    }
    return std::nullopt;
  }

  // parses a config line with the firmware's own parser
  static msgs::Config parseConfig(const std::string &line) {
    msgs::Config config;
    EXPECT_EQ(config.init(line), 0)
        << "'" << line << "': " << config.lastError();
    return config;
  }

  std::shared_ptr<rclcpp::Executor> executor_;
  std::unique_ptr<FakeEsp32> fake_;
  std::unique_ptr<MobileRobotHardware> hw_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> commands_;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> states_;
  // read()/write() ignore both, the controller manager passes them
  const rclcpp::Time now_{0, 0, RCL_STEADY_TIME};
  const rclcpp::Duration period_{10ms};
};

} // namespace mobile_robot_hardware::test

#endif
