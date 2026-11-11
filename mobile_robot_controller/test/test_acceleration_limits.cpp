// Component tests for MobileRobotController's acceleration limits (#83): the
// controller is driven through its lifecycle directly, with hand-made
// command/state interfaces standing in for the hardware (same pattern as
// test_cmd_vel_timeout.cpp). update() gets explicit times and periods; the
// limiter's dt is the period passed to update().

#include "mobile_robot_controller/kinematics.hpp"
#include "mobile_robot_controller/mobile_robot_controller.hpp"

#include <gtest/gtest.h>

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
using mobile_robot_controller::inverseKinematics;
using mobile_robot_controller::MobileRobotController;

namespace {

constexpr double k_eps = 1e-9;

const std::string k_controllerName = "mobile_robot_controller";
const std::string k_leftJoint = "left_wheel_joint";
const std::string k_rightJoint = "right_wheel_joint";

constexpr double k_separation = 0.2;
constexpr double k_radius = 0.05;

// values of mobile_robot_nodes/config/controller_manager.yaml and its 50 Hz
// update rate
constexpr double k_linearAccel = 0.3;
constexpr double k_linearDecel = 0.6;
constexpr double k_angularAccel = 2.0;
constexpr double k_angularDecel = 4.0;
constexpr double k_dt = 0.02;

constexpr double k_linearAccelStep = k_linearAccel * k_dt;   // 0.006
constexpr double k_linearDecelStep = k_linearDecel * k_dt;   // 0.012
constexpr double k_angularAccelStep = k_angularAccel * k_dt; // 0.04

// long enough that ramps finish before the command goes stale
constexpr double k_longTimeout = 5.0;

constexpr std::chrono::seconds k_waitTimeout{5};
constexpr double k_untouched = 123.0;

const std::vector<std::string> k_limitParameters = {
    "linear.x.max_acceleration",
    "linear.x.max_deceleration",
    "angular.z.max_acceleration",
    "angular.z.max_deceleration",
};

std::vector<rclcpp::Parameter> baseParameters(double timeout = k_longTimeout) {
  return {
      rclcpp::Parameter("left_wheel_joint_name", k_leftJoint),
      rclcpp::Parameter("right_wheel_joint_name", k_rightJoint),
      rclcpp::Parameter("wheel_separation", k_separation),
      rclcpp::Parameter("wheel_radius", k_radius),
      rclcpp::Parameter("position_variance_per_meter", 0.0025),
      rclcpp::Parameter("yaw_variance_per_radian", 0.01),
      rclcpp::Parameter("linear_velocity_variance", 0.0003),
      rclcpp::Parameter("angular_velocity_variance", 0.08),
      rclcpp::Parameter("cmd_vel_timeout", timeout),
  };
}

std::vector<rclcpp::Parameter>
limitedParameters(double timeout = k_longTimeout) {
  auto params = baseParameters(timeout);
  params.emplace_back("linear.x.max_acceleration", k_linearAccel);
  params.emplace_back("linear.x.max_deceleration", k_linearDecel);
  params.emplace_back("angular.z.max_acceleration", k_angularAccel);
  params.emplace_back("angular.z.max_deceleration", k_angularDecel);
  return params;
}

rclcpp::Time stamp(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)),
                      RCL_ROS_TIME);
}

rclcpp::Duration period(double seconds) {
  return rclcpp::Duration::from_seconds(seconds);
}

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

class AccelerationLimitTest : public ::testing::Test {
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

  void startController(const std::vector<rclcpp::Parameter> &params) {
    ASSERT_EQ(init(params), controller_interface::return_type::OK);
    ASSERT_TRUE(configure());

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
    ASSERT_TRUE(waitFor([&] {
      return cmdVelPub_->get_subscription_count() > 0;
    })) << "nothing subscribes to /cmd_vel";

    // replaces k_untouched by the zero command, so stepAfterCommand() starts
    // from a steady output
    step(0.0, 0.0);
  }

  double leftCommand() const { return leftVelCmd_->get_optional().value(); }
  double rightCommand() const { return rightVelCmd_->get_optional().value(); }

  // one control cycle at `time` with period `dt`
  void step(double time, double dt = k_dt) {
    ASSERT_EQ(controller_->update(stamp(time), period(dt)),
              controller_interface::return_type::OK);
  }

  // publishes the twist and runs update() at `time` until the wheel commands
  // change, i.e. until the controller has taken the first limited step
  // towards the new command. Only valid while the output is steady (the
  // previous command is reached and not stale), so updates before the
  // command arrives don't change the output.
  void stepAfterCommand(double time, double linear, double angular = 0.0,
                        double dt = k_dt) {
    const double leftBefore = leftCommand();
    const double rightBefore = rightCommand();

    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    cmdVelPub_->publish(msg);

    const bool changed = waitFor([&] {
      executor_->spin_some();
      if (controller_->update(stamp(time), period(dt)) !=
          controller_interface::return_type::OK) {
        return false;
      }
      return leftCommand() != leftBefore || rightCommand() != rightBefore;
    });
    ASSERT_TRUE(changed) << "wheel commands never changed, last: "
                         << leftCommand() << ", " << rightCommand();
  }

  // wheel commands match the body twist (m/s, rad/s)
  void expectTwist(double linear, double angular) const {
    const auto [left, right] =
        inverseKinematics(linear, angular, k_separation, k_radius);
    EXPECT_NEAR(leftCommand(), left, k_eps);
    EXPECT_NEAR(rightCommand(), right, k_eps);
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
// parameters
// ---------------------------------------------------------------------------

TEST_F(AccelerationLimitTest, LimitsDefaultToNaN) {
  ASSERT_EQ(init(baseParameters()), controller_interface::return_type::OK);
  ASSERT_TRUE(configure());
  for (const auto &name : k_limitParameters) {
    EXPECT_TRUE(
        std::isnan(controller_->get_node()->get_parameter(name).as_double()))
        << name;
  }
}

class AccelerationLimitInvalidTest
    : public AccelerationLimitTest,
      public ::testing::WithParamInterface<std::string> {};

TEST_P(AccelerationLimitInvalidTest, ConfigureFailsForNegativeLimit) {
  auto params = baseParameters();
  params.emplace_back(GetParam(), -0.1);
  ASSERT_EQ(init(params), controller_interface::return_type::OK);
  EXPECT_FALSE(configure());
}

INSTANTIATE_TEST_SUITE_P(Negative, AccelerationLimitInvalidTest,
                         ::testing::ValuesIn(k_limitParameters));

// ---------------------------------------------------------------------------
// ramps
// ---------------------------------------------------------------------------

// the case from #83: 0 -> 0.15 m/s at 0.3 m/s^2 takes 25 cycles of 0.006
TEST_F(AccelerationLimitTest, LinearStepRampsWithAccelerationLimit) {
  startController(limitedParameters());
  stepAfterCommand(10.0, 0.15);
  expectTwist(k_linearAccelStep, 0.0);

  for (int i = 1; i < 24; ++i) {
    step(10.0 + i * k_dt);
    expectTwist((i + 1) * k_linearAccelStep, 0.0);
  }
  step(10.0 + 24 * k_dt);
  expectTwist(0.15, 0.0);
  step(10.0 + 25 * k_dt);
  expectTwist(0.15, 0.0);
}

TEST_F(AccelerationLimitTest, PureRotationRampsWithAngularLimit) {
  startController(limitedParameters());
  stepAfterCommand(10.0, 0.0, 1.0);
  expectTwist(0.0, k_angularAccelStep);

  for (int i = 1; i < 24; ++i) {
    step(10.0 + i * k_dt);
    expectTwist(0.0, (i + 1) * k_angularAccelStep);
  }
  step(10.0 + 24 * k_dt);
  expectTwist(0.0, 1.0);
}

// no joint scaling: the angular part reaches its target after 13 cycles while
// the linear part keeps ramping
TEST_F(AccelerationLimitTest, LinearAndAngularAreLimitedIndependently) {
  startController(limitedParameters());
  stepAfterCommand(10.0, 0.15, 0.5);
  expectTwist(k_linearAccelStep, k_angularAccelStep);

  for (int i = 1; i < 13; ++i) {
    step(10.0 + i * k_dt);
  }
  expectTwist(13 * k_linearAccelStep, 0.5);
}

TEST_F(AccelerationLimitTest, LimitScalesWithUpdatePeriod) {
  startController(limitedParameters());
  stepAfterCommand(10.0, 0.15, 0.0, 0.04);
  expectTwist(0.3 * 0.04, 0.0);

  step(10.04, 0.01);
  expectTwist(0.3 * 0.04 + 0.3 * 0.01, 0.0);
}

TEST_F(AccelerationLimitTest, LowerCommandRampsDownWithDecelerationLimit) {
  startController(limitedParameters());
  stepAfterCommand(10.0, 0.15);
  for (int i = 1; i <= 25; ++i) {
    step(10.0 + i * k_dt);
  }
  expectTwist(0.15, 0.0);

  // 0.15 -> 0.05: 8 steps of 0.012, the 9th lands on 0.05
  stepAfterCommand(11.0, 0.05);
  expectTwist(0.15 - k_linearDecelStep, 0.0);
  for (int i = 1; i < 8; ++i) {
    step(11.0 + i * k_dt);
    expectTwist(0.15 - (i + 1) * k_linearDecelStep, 0.0);
  }
  step(11.0 + 8 * k_dt);
  expectTwist(0.05, 0.0);
}

// the timeout replaces a stale command by zero, which is reached with the
// deceleration limit instead of at once
TEST_F(AccelerationLimitTest, CmdVelTimeoutRampsDownWithDecelerationLimit) {
  startController(limitedParameters(1.0));
  stepAfterCommand(10.0, 0.15);
  for (int i = 1; i <= 50; ++i) {
    step(10.0 + i * k_dt);
  }
  // age 1.0 s = timeout: still applied
  expectTwist(0.15, 0.0);

  for (int i = 1; i <= 12; ++i) {
    step(11.0 + i * k_dt);
    expectTwist(0.15 - i * k_linearDecelStep, 0.0);
  }
  step(11.0 + 13 * k_dt);
  expectTwist(0.0, 0.0);
}

// the limited command held before a deactivation must not be the start of
// the ramp after the next activation
TEST_F(AccelerationLimitTest, ReactivationStartsRampFromZero) {
  startController(limitedParameters());
  stepAfterCommand(10.0, 0.15);
  for (int i = 1; i < 10; ++i) {
    step(10.0 + i * k_dt);
  }
  expectTwist(10 * k_linearAccelStep, 0.0);

  ASSERT_EQ(controller_->get_node()->deactivate().id(),
            State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(controller_->get_node()->activate().id(),
            State::PRIMARY_STATE_ACTIVE);
  step(11.0);
  expectTwist(0.0, 0.0);

  stepAfterCommand(11.02, 0.15);
  expectTwist(k_linearAccelStep, 0.0);
}

// ---------------------------------------------------------------------------
// unlimited
// ---------------------------------------------------------------------------

TEST_F(AccelerationLimitTest, WithoutLimitsStepPassesThrough) {
  startController(baseParameters());
  stepAfterCommand(10.0, 0.15, 1.0);
  expectTwist(0.15, 1.0);
}

TEST_F(AccelerationLimitTest, LinearLimitsLeaveAngularUnlimited) {
  auto params = baseParameters();
  params.emplace_back("linear.x.max_acceleration", k_linearAccel);
  params.emplace_back("linear.x.max_deceleration", k_linearDecel);
  startController(params);

  stepAfterCommand(10.0, 0.15, 1.0);
  expectTwist(k_linearAccelStep, 1.0);
}

TEST_F(AccelerationLimitTest, AngularLimitsLeaveLinearUnlimited) {
  auto params = baseParameters();
  params.emplace_back("angular.z.max_acceleration", k_angularAccel);
  params.emplace_back("angular.z.max_deceleration", k_angularDecel);
  startController(params);

  stepAfterCommand(10.0, 0.15, 1.0);
  expectTwist(0.15, k_angularAccelStep);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
