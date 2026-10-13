// component tests of the hardware plugin, driven through its lifecycle with a
// fake esp32 on a pty (see fake_esp32.hpp). The handshake state machine itself
// is covered by test_handshake.cpp, these check how the plugin uses it over
// the wire

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
#include <filesystem>
#include <memory>
#include <optional>
#include <ostream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using mobile_robot_hardware::MobileRobotHardware;
using mobile_robot_hardware::test::FakeEsp32;
using namespace std::chrono_literals;

namespace {

using Params = std::unordered_map<std::string, std::string>;

// the <param> tags of mobile_robot_nodes/description/ros2_control.xacro,
// keep in sync with it. serial_port is replaced by the pty in most tests
Params xacroDefaults() {
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

hardware_interface::InterfaceInfo makeInterface(const std::string &name) {
  hardware_interface::InterfaceInfo info{};
  info.name = name;
  info.data_type = "double";
  info.size = 1;
  info.enable_limits = false;
  return info;
}

hardware_interface::ComponentInfo
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
std::vector<hardware_interface::ComponentInfo> xacroJoints() {
  return {makeJoint("left_wheel_joint"), makeJoint("right_wheel_joint")};
}

// what the resource manager parses from the <ros2_control> block. Built by
// hand instead of from a URDF string, the parser would also need the links
// and joints of the whole robot
hardware_interface::HardwareInfo
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

rclcpp_lifecycle::State state(uint8_t id, const char *label) {
  return rclcpp_lifecycle::State(id, label);
}
const rclcpp_lifecycle::State k_unconfigured =
    state(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
          hardware_interface::lifecycle_state_names::UNCONFIGURED);
const rclcpp_lifecycle::State k_inactive =
    state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
          hardware_interface::lifecycle_state_names::INACTIVE);
const rclcpp_lifecycle::State k_active =
    state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
          hardware_interface::lifecycle_state_names::ACTIVE);

struct MotorCmd {
  float left;
  float right;
};

// "M,<left>,<right>"
std::optional<MotorCmd> parseMotorCmd(const std::string &line) {
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

} // namespace

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
    hw_.reset();
    fake_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  // runs what the resource manager does on loading a component: init() and
  // exporting the interfaces, which on_configure() looks up by name
  CallbackReturn init(const Params &params,
                      const std::vector<hardware_interface::ComponentInfo>
                          &joints = xacroJoints()) {
    hw_ = std::make_unique<MobileRobotHardware>();
    hardware_interface::HardwareComponentParams componentParams;
    componentParams.hardware_info = makeInfo(params, joints);
    componentParams.logger = rclcpp::get_logger("test_mobile_robot_hardware");
    componentParams.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    // without an executor no node is created, and with it no parameters
    componentParams.executor = executor_;
    const CallbackReturn ret = hw_->init(componentParams);
    if (ret == CallbackReturn::SUCCESS) {
      hw_->on_export_state_interfaces();
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
  // read()/write() ignore both, the controller manager passes them
  const rclcpp::Time now_{0, 0, RCL_STEADY_TIME};
  const rclcpp::Duration period_{10ms};
};

///////////////////////////////////////////////////////////////////////////////
// on_init
///////////////////////////////////////////////////////////////////////////////

// what the robot ships with must load
TEST_F(MobileRobotHardwareTest, InitAcceptsXacroDefaults) {
  EXPECT_EQ(init(xacroDefaults()), CallbackReturn::SUCCESS);
}

TEST_F(MobileRobotHardwareTest, InitAcceptsHandshakeTimeout) {
  Params params = xacroDefaults();
  params["handshake_timeout_ms"] = "1";
  EXPECT_EQ(init(params), CallbackReturn::SUCCESS);
}

// a hardware parameter set to a value, or removed if value is nullopt
struct ParamCase {
  const char *description;
  const char *name;
  std::optional<const char *> value;
};

void PrintTo(const ParamCase &c, std::ostream *os) { *os << c.description; }

Params applyCase(const ParamCase &c) {
  Params params = xacroDefaults();
  if (c.value) {
    params[c.name] = *c.value;
  } else {
    params.erase(c.name);
  }
  return params;
}

class InitParamTest : public MobileRobotHardwareTest,
                      public ::testing::WithParamInterface<ParamCase> {};

class InitRejectsParam : public InitParamTest {};

TEST_P(InitRejectsParam, InitFails) {
  EXPECT_EQ(init(applyCase(GetParam())), CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(
    MobileRobotHardware, InitRejectsParam,
    ::testing::Values(
        ParamCase{"missing serial_port", "serial_port", std::nullopt},
        ParamCase{"missing pid_kp", "pid_kp", std::nullopt},
        ParamCase{"missing invert_right", "invert_right", std::nullopt},
        ParamCase{"malformed pid_ki", "pid_ki", "abc"},
        ParamCase{"empty ff_slope_right", "ff_slope_right", ""},
        ParamCase{"zero baud_rate", "baud_rate", "0"},
        ParamCase{"negative baud_rate", "baud_rate", "-1"},
        // does not fit the uint16_t on the wire
        ParamCase{"watchdog_timeout_ms overflows", "watchdog_timeout_ms",
                  "65536"},
        // the esp32's range check, the pi must reject what it would NACK
        ParamCase{"watchdog_timeout_ms above max", "watchdog_timeout_ms",
                  "5001"},
        ParamCase{"zero control_rate_hz", "control_rate_hz", "0"},
        ParamCase{"report faster than control", "report_rate_hz", "201"},
        ParamCase{"negative pid_kp", "pid_kp", "-0.1"},
        ParamCase{"zero ff_slope_left", "ff_slope_left", "0"},
        ParamCase{"ff_min_duty_left of 1", "ff_min_duty_left", "1.0"},
        ParamCase{"zero ticks_per_wheel_rev", "ticks_per_wheel_rev", "0"},
        ParamCase{"zero handshake_timeout_ms", "handshake_timeout_ms", "0"},
        ParamCase{"negative handshake_timeout_ms", "handshake_timeout_ms",
                  "-5"},
        ParamCase{"malformed handshake_timeout_ms", "handshake_timeout_ms",
                  "soon"}));

class InitAcceptsParam : public InitParamTest {};

TEST_P(InitAcceptsParam, InitSucceeds) {
  EXPECT_EQ(init(applyCase(GetParam())), CallbackReturn::SUCCESS);
}

// the inclusive ends of the ranges, next to the rejected cases above
INSTANTIATE_TEST_SUITE_P(
    MobileRobotHardware, InitAcceptsParam,
    ::testing::Values(
        ParamCase{"watchdog_timeout_ms at max", "watchdog_timeout_ms", "5000"},
        ParamCase{"report as fast as control", "report_rate_hz", "200"},
        ParamCase{"zero pid_kp", "pid_kp", "0"},
        ParamCase{"ff_min_duty_left below 1", "ff_min_duty_left", "0.99"},
        ParamCase{"control_rate_hz at max", "control_rate_hz", "1000"},
        ParamCase{"report_rate_hz of 1", "report_rate_hz", "1"}));

TEST_F(MobileRobotHardwareTest, InitRejectsSingleJoint) {
  EXPECT_EQ(init(xacroDefaults(), {makeJoint("left_wheel_joint")}),
            CallbackReturn::ERROR);
}

TEST_F(MobileRobotHardwareTest, InitRejectsUnknownJointName) {
  EXPECT_EQ(init(xacroDefaults(),
                 {makeJoint("left_wheel_joint"), makeJoint("rear_joint")}),
            CallbackReturn::ERROR);
}

TEST_F(MobileRobotHardwareTest, InitRejectsPositionCommandInterface) {
  EXPECT_EQ(init(xacroDefaults(), {makeJoint("left_wheel_joint", {"position"}),
                                   makeJoint("right_wheel_joint")}),
            CallbackReturn::ERROR);
}

TEST_F(MobileRobotHardwareTest, InitRejectsMissingPositionState) {
  EXPECT_EQ(init(xacroDefaults(),
                 {makeJoint("left_wheel_joint"),
                  makeJoint("right_wheel_joint", {"velocity"}, {"velocity"})}),
            CallbackReturn::ERROR);
}

///////////////////////////////////////////////////////////////////////////////
// on_configure
///////////////////////////////////////////////////////////////////////////////

TEST_F(MobileRobotHardwareTest, ConfigureCompletesHandshake) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);

  EXPECT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::SUCCESS);
  EXPECT_TRUE(fake_->waitForLine("C,", k_wait).has_value());
  EXPECT_TRUE(fake_->slaveOpenedByPlugin());
}

// the esp32 rejects every attempt, e.g. a line that keeps getting corrupted
TEST_F(MobileRobotHardwareTest, ConfigureFailsWhenEspNacks) {
  fake_->setMode(FakeEsp32::Mode::NACK);
  ASSERT_EQ(
      init(fakeParams({{"handshake_timeout_ms", k_short_handshake_timeout}})),
      CallbackReturn::SUCCESS);

  EXPECT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::ERROR);
  // it got as far as sending the config
  EXPECT_TRUE(fake_->waitForLine("C,", k_wait).has_value());
}

// e.g. the esp32 is unflashed or hangs
TEST_F(MobileRobotHardwareTest, ConfigureFailsWhenEspIsSilent) {
  fake_->setMode(FakeEsp32::Mode::SILENT);
  ASSERT_EQ(
      init(fakeParams({{"handshake_timeout_ms", k_short_handshake_timeout}})),
      CallbackReturn::SUCCESS);

  EXPECT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::ERROR);
  EXPECT_TRUE(fake_->waitForLine("C,", k_wait).has_value());
}

// e.g. the esp32 is unplugged, so the udev symlink is missing
TEST_F(MobileRobotHardwareTest, ConfigureFailsOnMissingPort) {
  const auto missing = std::filesystem::temp_directory_path() /
                       "mobile_robot_hardware_test_no_such_port";
  ASSERT_FALSE(std::filesystem::exists(missing));
  ASSERT_EQ(init(fakeParams({{"serial_port", missing.string()}})),
            CallbackReturn::SUCCESS);

  EXPECT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::ERROR);
}

// the esp32 runs exactly what the pi was configured with
TEST_F(MobileRobotHardwareTest, ConfigLineCarriesParameters) {
  ASSERT_EQ(init(fakeParams({
                {"pid_kp", "0.75"},
                {"pid_ki", "1.25"},
                {"pid_integral_limit", "0.5"},
                {"ff_slope_left", "0.125"},
                {"ff_slope_right", "0.25"},
                {"ff_min_duty_left", "0.0625"},
                {"ff_min_duty_right", "0.03125"},
                {"ticks_per_wheel_rev", "1440"},
                {"watchdog_timeout_ms", "300"},
                {"control_rate_hz", "500"},
                {"report_rate_hz", "100"},
                {"invert_left", "false"},
                {"invert_right", "true"},
            })),
            CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::SUCCESS);

  const auto line = fake_->waitForLine("C,", k_wait);
  ASSERT_TRUE(line.has_value());
  const msgs::Config config = parseConfig(*line);
  constexpr float tol = 1e-6f;
  EXPECT_NE(config.handshake_id, 0u);
  EXPECT_NEAR(config.kp, 0.75f, tol);
  EXPECT_NEAR(config.ki, 1.25f, tol);
  EXPECT_NEAR(config.integral_limit, 0.5f, tol);
  EXPECT_NEAR(config.slope_left, 0.125f, tol);
  EXPECT_NEAR(config.slope_right, 0.25f, tol);
  EXPECT_NEAR(config.min_duty_left, 0.0625f, tol);
  EXPECT_NEAR(config.min_duty_right, 0.03125f, tol);
  EXPECT_NEAR(config.ticks_per_wheel_rev, 1440.0f, tol);
  EXPECT_EQ(config.watchdog_timeout_ms, 300u);
  EXPECT_EQ(config.control_rate_hz, 500u);
  EXPECT_EQ(config.report_rate_hz, 100u);
  EXPECT_FALSE(config.invert_left);
  EXPECT_TRUE(config.invert_right);
}

///////////////////////////////////////////////////////////////////////////////
// read() / write()
///////////////////////////////////////////////////////////////////////////////

// on_activate() resets the commands, so nothing stale moves the robot
TEST_F(MobileRobotHardwareTest, WriteSendsZeroAfterActivate) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();

  const auto cmd = writeAndReceive();
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->left, 0.0f, 1e-6f);
  EXPECT_NEAR(cmd->right, 0.0f, 1e-6f);
}

TEST_F(MobileRobotHardwareTest, WriteSendsCommandOnceConfigured) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();
  setCommand(1.5, -2.0);

  const auto cmd = writeAndReceive();
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->left, 1.5f, 1e-6f);
  EXPECT_NEAR(cmd->right, -2.0f, 1e-6f);
}

// a restarted esp32 has lost its configuration and asks for a new one
TEST_F(MobileRobotHardwareTest, EspRestartStartsHandshakeWithHigherId) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();
  const auto first = fake_->waitForLine("C,", k_wait);
  ASSERT_TRUE(first.has_value());
  setCommand(1.5, -2.0);

  // hold the ACK back to see what is written in between
  fake_->setMode(FakeEsp32::Mode::SILENT);
  fake_->sendLine("S,SETUP");
  const auto second = readUntilFakeReceives("C,");
  ASSERT_TRUE(second.has_value());
  // the esp32 tells the handshakes apart by their id
  EXPECT_GT(FakeEsp32::configId(*second), FakeEsp32::configId(*first));

  // the esp32 ignores motor commands until configured
  const auto beforeAck = writeAndReceive();
  ASSERT_TRUE(beforeAck.has_value());
  EXPECT_NEAR(beforeAck->left, 0.0f, 1e-6f);
  EXPECT_NEAR(beforeAck->right, 0.0f, 1e-6f);

  fake_->sendLine("S,ACK," + std::to_string(FakeEsp32::configId(*second)));
  const auto afterAck = cycleUntilNonZeroCmd();
  ASSERT_TRUE(afterAck.has_value());
  EXPECT_NEAR(afterAck->left, 1.5f, 1e-6f);
  EXPECT_NEAR(afterAck->right, -2.0f, 1e-6f);
}

TEST_F(MobileRobotHardwareTest, LiveParameterChangeStartsHandshake) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();
  setCommand(1.5, -2.0);
  fake_->setMode(FakeEsp32::Mode::SILENT);
  fake_->discardReceived();

  const auto result =
      hw_->get_node()->set_parameter(rclcpp::Parameter("pid_kp", 0.9));
  ASSERT_TRUE(result.successful) << result.reason;

  // read() starts the handshake and sends the new value right away
  ASSERT_EQ(hw_->read(now_, period_), return_type::OK);
  const auto line = fake_->nextLine(k_wait);
  ASSERT_TRUE(line.has_value());
  ASSERT_EQ(line->rfind("C,", 0), 0u) << *line;
  const msgs::Config config = parseConfig(*line);
  EXPECT_NEAR(config.kp, 0.9f, 1e-6f);

  // zero until the esp32 runs the new values
  const auto beforeAck = writeAndReceive();
  ASSERT_TRUE(beforeAck.has_value());
  EXPECT_NEAR(beforeAck->left, 0.0f, 1e-6f);
  EXPECT_NEAR(beforeAck->right, 0.0f, 1e-6f);

  fake_->sendLine("S,ACK," + std::to_string(config.handshake_id));
  const auto afterAck = cycleUntilNonZeroCmd();
  ASSERT_TRUE(afterAck.has_value());
  EXPECT_NEAR(afterAck->left, 1.5f, 1e-6f);
  EXPECT_NEAR(afterAck->right, -2.0f, 1e-6f);
}

// a value the esp32 would NACK must not reach it and must not stop the robot
TEST_F(MobileRobotHardwareTest, InvalidLiveParameterIsRejected) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();
  setCommand(1.5, -2.0);
  fake_->discardReceived();

  const auto result =
      hw_->get_node()->set_parameter(rclcpp::Parameter("ff_slope_left", 0.0));
  EXPECT_FALSE(result.successful);
  // declared from the float in the payload, so not exactly 0.1
  EXPECT_NEAR(hw_->get_node()->get_parameter("ff_slope_left").as_double(), 0.1,
              1e-6);

  // a handshake would send its config line from read(), before the write()
  ASSERT_EQ(hw_->read(now_, period_), return_type::OK);
  ASSERT_EQ(hw_->write(now_, period_), return_type::OK);
  const auto line = fake_->nextLine(k_wait);
  ASSERT_TRUE(line.has_value());
  const auto cmd = parseMotorCmd(*line);
  ASSERT_TRUE(cmd.has_value()) << "expected a motor command, got " << *line;
  EXPECT_NEAR(cmd->left, 1.5f, 1e-6f);
  EXPECT_NEAR(cmd->right, -2.0f, 1e-6f);
}

// controller_manager then takes the component down (OVERVIEW.md)
TEST_F(MobileRobotHardwareTest, FailedHandshakeWhileActiveMakesReadFail) {
  ASSERT_EQ(
      init(fakeParams({{"handshake_timeout_ms", k_short_handshake_timeout}})),
      CallbackReturn::SUCCESS);
  configureAndActivate();
  fake_->setMode(FakeEsp32::Mode::SILENT);
  fake_->sendLine("S,SETUP");

  // bounded by the handshake timeout plus a generous margin
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  return_type ret = return_type::OK;
  while (ret == return_type::OK &&
         std::chrono::steady_clock::now() < deadline) {
    ret = hw_->read(now_, period_);
    std::this_thread::yield();
  }
  EXPECT_EQ(ret, return_type::ERROR);
  // the config was sent, so the failure is the timeout and not the port
  EXPECT_TRUE(fake_->waitForLine("C,", k_wait).has_value());
}

///////////////////////////////////////////////////////////////////////////////
// on_deactivate / on_cleanup / on_shutdown
///////////////////////////////////////////////////////////////////////////////

// INACTIVE keeps the port open: read() still runs there
TEST_F(MobileRobotHardwareTest, DeactivateStopsMotorsAndKeepsPortOpen) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();
  setCommand(1.5, -2.0);
  fake_->discardReceived();

  EXPECT_EQ(hw_->on_deactivate(k_active), CallbackReturn::SUCCESS);
  const auto line = fake_->waitForLine("M,", k_wait);
  ASSERT_TRUE(line.has_value());
  const auto cmd = parseMotorCmd(*line);
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->left, 0.0f, 1e-6f);
  EXPECT_NEAR(cmd->right, 0.0f, 1e-6f);
  EXPECT_TRUE(fake_->slaveOpenedByPlugin());
}

TEST_F(MobileRobotHardwareTest, CleanupClosesPort) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::SUCCESS);
  ASSERT_TRUE(fake_->slaveOpenedByPlugin());

  EXPECT_EQ(hw_->on_cleanup(k_inactive), CallbackReturn::SUCCESS);
  EXPECT_FALSE(fake_->slaveOpenedByPlugin());
}

// e.g. after the esp32 was unplugged and plugged back in
TEST_F(MobileRobotHardwareTest, ReconfigureAfterCleanup) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::SUCCESS);
  ASSERT_EQ(hw_->on_cleanup(k_inactive), CallbackReturn::SUCCESS);
  fake_->discardReceived();

  EXPECT_EQ(hw_->on_configure(k_unconfigured), CallbackReturn::SUCCESS);
  EXPECT_TRUE(fake_->waitForLine("C,", k_wait).has_value());
  EXPECT_TRUE(fake_->slaveOpenedByPlugin());

  ASSERT_EQ(hw_->on_activate(k_inactive), CallbackReturn::SUCCESS);
  setCommand(0.5, 0.25);
  const auto cmd = writeAndReceive();
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->left, 0.5f, 1e-6f);
  EXPECT_NEAR(cmd->right, 0.25f, 1e-6f);
}

// shutdown can come straight from ACTIVE, it has to stop and close itself
TEST_F(MobileRobotHardwareTest, ShutdownFromActiveStopsMotorsAndClosesPort) {
  ASSERT_EQ(init(fakeParams()), CallbackReturn::SUCCESS);
  configureAndActivate();
  setCommand(1.5, -2.0);
  fake_->discardReceived();

  EXPECT_EQ(hw_->on_shutdown(k_active), CallbackReturn::SUCCESS);
  const auto line = fake_->waitForLine("M,", k_wait);
  ASSERT_TRUE(line.has_value());
  const auto cmd = parseMotorCmd(*line);
  ASSERT_TRUE(cmd.has_value());
  EXPECT_NEAR(cmd->left, 0.0f, 1e-6f);
  EXPECT_NEAR(cmd->right, 0.0f, 1e-6f);
  EXPECT_FALSE(fake_->slaveOpenedByPlugin());
}
