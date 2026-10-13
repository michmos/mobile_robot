// Component tests for MobileRobotController's cmd_vel timeout (#40): the
// controller is driven through its lifecycle directly, with hand-made
// command/state interfaces standing in for the hardware (the pattern of
// ros2_controllers' diff_drive_controller tests). update() gets explicit
// times, so the timeout is checked without depending on the wall clock.

#include "mobile_robot_controller/mobile_robot_controller.hpp"

#include <gtest/gtest.h>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;
using mobile_robot_controller::MobileRobotController;

namespace {

constexpr double k_eps = 1e-9;

const std::string k_controllerName = "mobile_robot_controller";
const std::string k_leftJoint = "left_wheel_joint";
const std::string k_rightJoint = "right_wheel_joint";

// round numbers: 0.1 m/s straight ahead -> 2 rad/s on both wheels
constexpr double k_separation = 0.2;
constexpr double k_radius = 0.05;
constexpr double k_linear = 0.1;
constexpr double k_wheelVel = k_linear / k_radius;

// default of the cmd_vel_timeout parameter
constexpr double k_defaultTimeout = 0.5;

// bounded wait for DDS discovery / message delivery inside this process -
// only reached if something is broken
constexpr std::chrono::seconds k_waitTimeout{5};

// a clearly non-zero value the command interfaces start with, so a zero
// written by the controller can be told apart from "never written"
constexpr double k_untouched = 123.0;

std::vector<rclcpp::Parameter> validParameters() {
  return {
      rclcpp::Parameter("left_wheel_joint_name", k_leftJoint),
      rclcpp::Parameter("right_wheel_joint_name", k_rightJoint),
      rclcpp::Parameter("wheel_separation", k_separation),
      rclcpp::Parameter("wheel_radius", k_radius),
      rclcpp::Parameter("position_variance_per_meter", 0.0025),
      rclcpp::Parameter("yaw_variance_per_radian", 0.01),
      rclcpp::Parameter("linear_velocity_variance", 0.0003),
      rclcpp::Parameter("angular_velocity_variance", 0.08),
  };
}

rclcpp::Time stamp(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)),
                      RCL_ROS_TIME);
}

rclcpp::Duration period(double seconds) {
  return rclcpp::Duration::from_seconds(seconds);
}

// polls `condition` until it holds or k_waitTimeout passed
// @return: whether the condition held in time
bool waitFor(const std::function<bool()> &condition) {
  const auto deadline = std::chrono::steady_clock::now() + k_waitTimeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

} // namespace

class CmdVelTimeoutTest : public ::testing::Test {
protected:
  void SetUp() override {
    controller_ = std::make_unique<MobileRobotController>();

    leftPosState_ = std::make_shared<hardware_interface::StateInterface>(
        k_leftJoint, HW_IF_POSITION);
    rightPosState_ = std::make_shared<hardware_interface::StateInterface>(
        k_rightJoint, HW_IF_POSITION);
    leftVelCmd_ = std::make_shared<hardware_interface::CommandInterface>(
        k_leftJoint, HW_IF_VELOCITY);
    rightVelCmd_ = std::make_shared<hardware_interface::CommandInterface>(
        k_rightJoint, HW_IF_VELOCITY);
    std::ignore = leftPosState_->set_value(0.0);
    std::ignore = rightPosState_->set_value(0.0);
    std::ignore = leftVelCmd_->set_value(k_untouched);
    std::ignore = rightVelCmd_->set_value(k_untouched);

    testNode_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override {
    // the loaned interfaces reference the interfaces above, so the
    // controller has to go first
    executor_.reset();
    controller_.reset();
  }

  controller_interface::return_type
  init(const std::vector<rclcpp::Parameter> &params) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(params);
    controller_interface::ControllerInterfaceParams initParams;
    initParams.controller_name = k_controllerName;
    initParams.robot_description = "";
    initParams.update_rate = 0;
    initParams.node_namespace = "";
    initParams.node_options = options;
    return controller_->init(initParams);
  }

  bool configure() {
    return controller_->configure().id() == State::PRIMARY_STATE_INACTIVE;
  }

  // init + configure + assign interfaces + activate, plus an executor
  // spinning the controller's node and the test node
  void startController(
      const std::vector<rclcpp::Parameter> &params = validParameters()) {
    ASSERT_EQ(init(params), controller_interface::return_type::OK);
    ASSERT_TRUE(configure());

    // same order as *_interface_configuration(), like controller_manager
    std::vector<hardware_interface::LoanedCommandInterface> commandInterfaces;
    commandInterfaces.emplace_back(leftVelCmd_, nullptr);
    commandInterfaces.emplace_back(rightVelCmd_, nullptr);
    std::vector<hardware_interface::LoanedStateInterface> stateInterfaces;
    stateInterfaces.emplace_back(leftPosState_, nullptr);
    stateInterfaces.emplace_back(rightPosState_, nullptr);
    controller_->assign_interfaces(std::move(commandInterfaces),
                                   std::move(stateInterfaces));

    ASSERT_EQ(controller_->get_node()->activate().id(),
              State::PRIMARY_STATE_ACTIVE);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(testNode_);

    cmdVelPub_ = testNode_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10);
    ASSERT_TRUE(waitFor([&] { return cmdVelPub_->get_subscription_count() > 0; }))
        << "nothing subscribes to /cmd_vel";
  }

  double leftCommand() const { return leftVelCmd_->get_optional().value(); }
  double rightCommand() const { return rightVelCmd_->get_optional().value(); }

  void update(double time) {
    ASSERT_EQ(controller_->update(stamp(time), period(0.0)),
              controller_interface::return_type::OK);
  }

  // publishes `linear` straight ahead and runs update() at `time` until the
  // controller has picked it up. All updates use the same time, so `time`
  // is the time the controller first saw the command.
  void sendCommand(double time, double linear) {
    sendCommand(time, linear, builtin_interfaces::msg::Time());
  }

  void sendCommand(double time, double linear,
                   const builtin_interfaces::msg::Time &headerStamp) {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = headerStamp;
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = linear;
    cmdVelPub_->publish(msg);

    const double expected = linear / k_radius;
    const bool received = waitFor([&] {
      executor_->spin_some();
      if (controller_->update(stamp(time), period(0.0)) !=
          controller_interface::return_type::OK) {
        return false;
      }
      return std::abs(leftCommand() - expected) < k_eps &&
             std::abs(rightCommand() - expected) < k_eps;
    });
    ASSERT_TRUE(received) << "wheel commands never matched the twist, last: "
                          << leftCommand() << ", " << rightCommand();
  }

  void expectWheelCommands(double expected) const {
    EXPECT_NEAR(leftCommand(), expected, k_eps);
    EXPECT_NEAR(rightCommand(), expected, k_eps);
  }

  std::unique_ptr<MobileRobotController> controller_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  hardware_interface::StateInterface::SharedPtr leftPosState_;
  hardware_interface::StateInterface::SharedPtr rightPosState_;
  hardware_interface::CommandInterface::SharedPtr leftVelCmd_;
  hardware_interface::CommandInterface::SharedPtr rightVelCmd_;

  rclcpp::Node::SharedPtr testNode_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelPub_;
};

// ---------------------------------------------------------------------------
// parameter
// ---------------------------------------------------------------------------

class CmdVelTimeoutInvalidTest : public CmdVelTimeoutTest,
                                 public ::testing::WithParamInterface<double> {
};

// 0 would stop the robot every cycle, a negative value makes no sense - and
// neither may silently mean "no timeout"
TEST_P(CmdVelTimeoutInvalidTest, ConfigureFailsForNonPositiveTimeout) {
  auto params = validParameters();
  params.emplace_back("cmd_vel_timeout", GetParam());
  ASSERT_EQ(init(params), controller_interface::return_type::OK);
  EXPECT_FALSE(configure());
}

INSTANTIATE_TEST_SUITE_P(NonPositive, CmdVelTimeoutInvalidTest,
                         ::testing::Values(0.0, -0.5));

TEST_F(CmdVelTimeoutTest, DefaultTimeoutIsHalfASecond) {
  startController();
  EXPECT_NEAR(controller_->get_node()->get_parameter("cmd_vel_timeout").as_double(),
              k_defaultTimeout, k_eps);
}

// ---------------------------------------------------------------------------
// timeout behavior
// ---------------------------------------------------------------------------

TEST_F(CmdVelTimeoutTest, FreshCommandPassesThrough) {
  startController();
  sendCommand(10.0, k_linear);

  update(10.0 + 0.2);
  expectWheelCommands(k_wheelVel);
}

// boundary: a command exactly as old as the timeout is still applied, only
// an older one is stale
TEST_F(CmdVelTimeoutTest, CommandExactlyAtTimeoutIsStillApplied) {
  startController();
  sendCommand(10.0, k_linear);

  update(10.0 + k_defaultTimeout);
  expectWheelCommands(k_wheelVel);
}

// the bug from #40: the velocity source stops publishing and the robot must
// not keep driving with the last command
TEST_F(CmdVelTimeoutTest, StaleCommandGivesZero) {
  startController();
  sendCommand(10.0, k_linear);

  update(10.0 + k_defaultTimeout + 0.001);
  expectWheelCommands(0.0);
}

TEST_F(CmdVelTimeoutTest, CommandStaysZeroWhileNoNewCommandArrives) {
  startController();
  sendCommand(10.0, k_linear);

  update(11.0);
  update(20.0);
  expectWheelCommands(0.0);
}

TEST_F(CmdVelTimeoutTest, NewCommandAfterTimeoutResumes) {
  startController();
  sendCommand(10.0, k_linear);
  update(11.0);
  expectWheelCommands(0.0);

  sendCommand(12.0, 2.0 * k_linear);
  update(12.0 + 0.2);
  expectWheelCommands(2.0 * k_wheelVel);
}

// a steady stream of commands (e.g. joystick at 20 Hz) keeps the robot
// moving, even long after the first command
TEST_F(CmdVelTimeoutTest, RepeatedCommandsKeepRobotMoving) {
  startController();
  sendCommand(10.0, k_linear);
  sendCommand(10.4, k_linear);
  sendCommand(10.8, k_linear);

  update(10.8 + 0.4);
  expectWheelCommands(k_wheelVel);
}

TEST_F(CmdVelTimeoutTest, TimeoutIsConfigurable) {
  auto params = validParameters();
  params.emplace_back("cmd_vel_timeout", 2.0);
  startController(params);
  sendCommand(10.0, k_linear);

  update(11.5);
  expectWheelCommands(k_wheelVel);
  update(12.1);
  expectWheelCommands(0.0);
}

// header stamps can be 0 (twist_mux / teleop often leave them unset) or come
// from another machine's clock - only the time the controller saw the
// command counts
TEST_F(CmdVelTimeoutTest, HeaderStampIsIgnored) {
  startController();
  builtin_interfaces::msg::Time zero;
  sendCommand(10.0, k_linear, zero);
  update(10.2);
  expectWheelCommands(k_wheelVel);

  builtin_interfaces::msg::Time future;
  future.sec = 1000;
  sendCommand(11.0, 2.0 * k_linear, future);
  update(11.0 + k_defaultTimeout + 0.001);
  expectWheelCommands(0.0);
}

// if the update time jumps back (e.g. a simulation reset), the age of the
// held command is unknown - stopping is the safe choice
TEST_F(CmdVelTimeoutTest, TimeJumpingBackGivesZero) {
  startController();
  sendCommand(10.0, k_linear);

  update(9.0);
  expectWheelCommands(0.0);
}

// the command seen before a deactivation must not be held over into the
// next activation
TEST_F(CmdVelTimeoutTest, ReactivationStartsWithZeroCommand) {
  startController();
  sendCommand(10.0, k_linear);

  ASSERT_EQ(controller_->get_node()->deactivate().id(),
            State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(controller_->get_node()->activate().id(),
            State::PRIMARY_STATE_ACTIVE);
  update(10.1);
  expectWheelCommands(0.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
