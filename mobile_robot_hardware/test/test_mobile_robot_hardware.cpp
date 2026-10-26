// component tests of the hardware plugin, driven through its lifecycle with a
// fake esp32 on a pty (see fake_esp32.hpp). The handshake state machine itself
// is covered by test_handshake.cpp, these check how the plugin uses it over
// the wire

#include "hardware_fixture.hpp"

#include <gtest/gtest.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace mobile_robot_hardware::test;

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
        ParamCase{"missing invert_encoder_right", "invert_encoder_right",
                  std::nullopt},
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
                {"invert_motor_left", "false"},
                {"invert_motor_right", "true"},
                {"invert_encoder_left", "true"},
                {"invert_encoder_right", "false"},
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
  EXPECT_FALSE(config.invert_motor_left);
  EXPECT_TRUE(config.invert_motor_right);
  EXPECT_TRUE(config.invert_encoder_left);
  EXPECT_FALSE(config.invert_encoder_right);
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
