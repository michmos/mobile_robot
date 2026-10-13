// Component tests for MobileRobotController: the controller is driven through
// its lifecycle directly (init -> configure -> activate -> update), with
// hand-made command/state interfaces standing in for the hardware - the same
// pattern as ros2_controllers' diff_drive_controller tests.

#include "mobile_robot_controller/mobile_robot_controller.hpp"

#include <gmock/gmock.h>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
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
using testing::ElementsAre;

namespace {

constexpr double k_pi = M_PI;
constexpr double k_eps = 1e-9;

// same name as in controller_manager.yaml, so the node (and its topics)
// resolve exactly like on the robot
const std::string k_controllerName = "mobile_robot_controller";
const std::string k_leftJoint = "left_wheel_joint";
const std::string k_rightJoint = "right_wheel_joint";

// round numbers for hand-checked cases
constexpr double k_separation = 0.2;
constexpr double k_radius = 0.05;

// real robot, see mobile_robot_nodes/config/controller_manager.yaml
constexpr double k_positionVariancePerMeter = 0.0025;
constexpr double k_yawVariancePerRadian = 0.01;
constexpr double k_linearVelocityVariance = 0.0003;
constexpr double k_angularVelocityVariance = 0.08;

// variance the controller uses for the axes a planar robot cannot move
constexpr double k_negligibleVariance = 1e-9;

// bounded wait for DDS discovery / message delivery between nodes of this
// process - only reached if something is broken
constexpr std::chrono::seconds k_timeout{5};

// a clearly non-zero value the hardware interfaces start with, so a zero
// written by the controller can be told apart from "never written"
constexpr double k_untouched = 123.0;

std::vector<rclcpp::Parameter> validParameters() {
  return {
      rclcpp::Parameter("left_wheel_joint_name", k_leftJoint),
      rclcpp::Parameter("right_wheel_joint_name", k_rightJoint),
      rclcpp::Parameter("wheel_separation", k_separation),
      rclcpp::Parameter("wheel_radius", k_radius),
      rclcpp::Parameter("position_variance_per_meter",
                        k_positionVariancePerMeter),
      rclcpp::Parameter("yaw_variance_per_radian", k_yawVariancePerRadian),
      rclcpp::Parameter("linear_velocity_variance", k_linearVelocityVariance),
      rclcpp::Parameter("angular_velocity_variance",
                        k_angularVelocityVariance),
  };
}

// replaces (or adds) one parameter of `params`
std::vector<rclcpp::Parameter>
withParameter(std::vector<rclcpp::Parameter> params,
              const rclcpp::Parameter &override) {
  for (auto &param : params) {
    if (param.get_name() == override.get_name()) {
      param = override;
      return params;
    }
  }
  params.push_back(override);
  return params;
}

rclcpp::Time stamp(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

rclcpp::Duration period(double seconds) {
  return rclcpp::Duration::from_seconds(seconds);
}

// polls `condition` until it holds or k_timeout passed
// @return: whether the condition held in time
bool waitFor(const std::function<bool()> &condition) {
  const auto deadline = std::chrono::steady_clock::now() + k_timeout;
  while (!condition()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

} // namespace

class MobileRobotControllerTest : public ::testing::Test {
protected:
  void SetUp() override {
    controller_ = std::make_unique<MobileRobotController>();

    leftPosState_ =
        std::make_shared<hardware_interface::StateInterface>(k_leftJoint, HW_IF_POSITION);
    rightPosState_ =
        std::make_shared<hardware_interface::StateInterface>(k_rightJoint, HW_IF_POSITION);
    leftVelCmd_ =
        std::make_shared<hardware_interface::CommandInterface>(k_leftJoint, HW_IF_VELOCITY);
    rightVelCmd_ =
        std::make_shared<hardware_interface::CommandInterface>(k_rightJoint, HW_IF_VELOCITY);
    setWheelPositions(0.0, 0.0);
    std::ignore = leftVelCmd_->set_value(k_untouched);
    std::ignore = rightVelCmd_->set_value(k_untouched);

    testNode_ = std::make_shared<rclcpp::Node>("test_node");
  }

  void TearDown() override {
    // the loaned interfaces hold references to the interfaces above, so the
    // controller has to go first
    executor_.reset();
    controller_.reset();
  }

  controller_interface::return_type
  init(const std::vector<rclcpp::Parameter> &params = validParameters()) {
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

  // claims the interfaces in the order of *_interface_configuration(), like
  // the controller_manager does
  void assignInterfaces() {
    std::vector<hardware_interface::LoanedCommandInterface> commandInterfaces;
    commandInterfaces.emplace_back(leftVelCmd_, nullptr);
    commandInterfaces.emplace_back(rightVelCmd_, nullptr);
    std::vector<hardware_interface::LoanedStateInterface> stateInterfaces;
    stateInterfaces.emplace_back(leftPosState_, nullptr);
    stateInterfaces.emplace_back(rightPosState_, nullptr);
    controller_->assign_interfaces(std::move(commandInterfaces),
                                   std::move(stateInterfaces));
  }

  bool activate() {
    return controller_->get_node()->activate().id() ==
           State::PRIMARY_STATE_ACTIVE;
  }

  bool deactivate() {
    return controller_->get_node()->deactivate().id() ==
           State::PRIMARY_STATE_INACTIVE;
  }

  // init + configure + assign + activate with valid parameters, plus an
  // executor spinning the controller's node and the test node
  void startController() {
    ASSERT_EQ(init(), controller_interface::return_type::OK);
    ASSERT_TRUE(configure());
    assignInterfaces();
    ASSERT_TRUE(activate());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(controller_->get_node()->get_node_base_interface());
    executor_->add_node(testNode_);
  }

  void setWheelPositions(double left, double right) {
    ASSERT_TRUE(leftPosState_->set_value(left));
    ASSERT_TRUE(rightPosState_->set_value(right));
  }

  double leftCommand() const { return leftVelCmd_->get_optional().value(); }
  double rightCommand() const { return rightVelCmd_->get_optional().value(); }

  void update(double time, double dt) {
    ASSERT_EQ(controller_->update(stamp(time), period(dt)),
              controller_interface::return_type::OK);
  }

  // publishes a twist on the global topic twist_mux writes to (see
  // mobile_robot_launch.py) and waits until the controller has picked it up,
  // i.e. until an update() writes the matching wheel commands
  void sendCommand(double linear, double angular, double expectedLeft,
                   double expectedRight) {
    auto pub = testNode_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 10);
    ASSERT_TRUE(waitFor([&] { return pub->get_subscription_count() > 0; }))
        << "nothing subscribes to /cmd_vel";

    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = linear;
    msg.twist.angular.z = angular;
    pub->publish(msg);

    // dt == 0 and unchanged wheel positions: these updates leave the
    // odometry untouched
    const bool received = waitFor([&] {
      executor_->spin_some();
      if (controller_->update(stamp(0.0), period(0.0)) !=
          controller_interface::return_type::OK) {
        return false;
      }
      return std::abs(leftCommand() - expectedLeft) < k_eps &&
             std::abs(rightCommand() - expectedRight) < k_eps;
    });
    ASSERT_TRUE(received) << "wheel commands never matched the twist, last: "
                          << leftCommand() << ", " << rightCommand();
  }

  // repeats a no-motion, dt == 0 update at `time` until the odom and tf
  // messages of that update arrive - the realtime publishers drop a message
  // while the previous one is still being sent, so a single update() is not
  // guaranteed to come through
  void publishAndReceive(double time) {
    const rclcpp::Time t = stamp(time);
    ASSERT_TRUE(waitFor([&] {
      if (controller_->update(t, period(0.0)) !=
          controller_interface::return_type::OK) {
        return false;
      }
      executor_->spin_some();
      return lastOdom_ && lastTf_ && rclcpp::Time(lastOdom_->header.stamp) == t &&
             !lastTf_->transforms.empty() &&
             rclcpp::Time(lastTf_->transforms[0].header.stamp) == t;
    })) << "no odom/tf message for t = " << time << " arrived";
  }

  void subscribeOdometry() {
    odomSub_ = testNode_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          lastOdom_ = msg;
        });
    tfSub_ = testNode_->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", 10, [this](tf2_msgs::msg::TFMessage::SharedPtr msg) {
          lastTf_ = msg;
        });
  }

  void expectPose(double x, double y, double heading) const {
    ASSERT_TRUE(lastOdom_);
    ASSERT_TRUE(lastTf_);
    ASSERT_EQ(lastTf_->transforms.size(), 1u);
    const auto &pose = lastOdom_->pose.pose;
    const auto &tf = lastTf_->transforms[0].transform;

    EXPECT_NEAR(pose.position.x, x, k_eps);
    EXPECT_NEAR(pose.position.y, y, k_eps);
    EXPECT_NEAR(pose.position.z, 0.0, k_eps);
    EXPECT_NEAR(pose.orientation.x, 0.0, k_eps);
    EXPECT_NEAR(pose.orientation.y, 0.0, k_eps);
    EXPECT_NEAR(pose.orientation.z, std::sin(heading / 2.0), k_eps);
    EXPECT_NEAR(pose.orientation.w, std::cos(heading / 2.0), k_eps);

    // tf has to agree with odom, otherwise e.g. rviz and nav2 disagree on
    // where the robot is
    EXPECT_NEAR(tf.translation.x, x, k_eps);
    EXPECT_NEAR(tf.translation.y, y, k_eps);
    EXPECT_NEAR(tf.translation.z, 0.0, k_eps);
    EXPECT_NEAR(tf.rotation.x, pose.orientation.x, k_eps);
    EXPECT_NEAR(tf.rotation.y, pose.orientation.y, k_eps);
    EXPECT_NEAR(tf.rotation.z, pose.orientation.z, k_eps);
    EXPECT_NEAR(tf.rotation.w, pose.orientation.w, k_eps);
  }

  void expectTwist(double linear, double angular) const {
    ASSERT_TRUE(lastOdom_);
    const auto &twist = lastOdom_->twist.twist;
    EXPECT_NEAR(twist.linear.x, linear, k_eps);
    EXPECT_NEAR(twist.linear.y, 0.0, k_eps);
    EXPECT_NEAR(twist.angular.z, angular, k_eps);
  }

  std::unique_ptr<MobileRobotController> controller_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  hardware_interface::StateInterface::SharedPtr leftPosState_;
  hardware_interface::StateInterface::SharedPtr rightPosState_;
  hardware_interface::CommandInterface::SharedPtr leftVelCmd_;
  hardware_interface::CommandInterface::SharedPtr rightVelCmd_;

  rclcpp::Node::SharedPtr testNode_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tfSub_;
  nav_msgs::msg::Odometry::SharedPtr lastOdom_;
  tf2_msgs::msg::TFMessage::SharedPtr lastTf_;
};

// ---------------------------------------------------------------------------
// parameters (on_init / on_configure)
// ---------------------------------------------------------------------------

TEST_F(MobileRobotControllerTest, ConfigureSucceedsWithValidParameters) {
  ASSERT_EQ(init(), controller_interface::return_type::OK);
  EXPECT_TRUE(configure());
}

TEST_F(MobileRobotControllerTest, ConfigureFailsWithoutParameters) {
  // nothing set: every parameter keeps its declared default (empty name, 0)
  ASSERT_EQ(init({}), controller_interface::return_type::OK);
  EXPECT_FALSE(configure());
}

TEST_F(MobileRobotControllerTest, ConfigureSucceedsWithSmallestPositiveValues) {
  // the check is "> 0", so any positive value has to pass
  auto params = validParameters();
  for (const char *name :
       {"wheel_separation", "wheel_radius", "position_variance_per_meter",
        "yaw_variance_per_radian", "linear_velocity_variance",
        "angular_velocity_variance"}) {
    params = withParameter(params, rclcpp::Parameter(name, 1e-12));
  }
  ASSERT_EQ(init(params), controller_interface::return_type::OK);
  EXPECT_TRUE(configure());
}

TEST_F(MobileRobotControllerTest, InitFailsOnWrongParameterType) {
  // e.g. a typo in the yaml that turns the number into a string
  ASSERT_EQ(init(withParameter(validParameters(),
                               rclcpp::Parameter("wheel_radius", "0.0335"))),
            controller_interface::return_type::ERROR);
}

TEST_F(MobileRobotControllerTest, ConfigureCanBeRetriedAfterFailure) {
  // a rejected configure must leave the controller unconfigured, not broken:
  // fixing the parameter and configuring again works
  ASSERT_EQ(init(withParameter(validParameters(),
                               rclcpp::Parameter("wheel_radius", 0.0))),
            controller_interface::return_type::OK);
  ASSERT_FALSE(configure());

  controller_->get_node()->set_parameter(rclcpp::Parameter("wheel_radius", k_radius));
  EXPECT_TRUE(configure());
}

class InvalidParameterTest
    : public MobileRobotControllerTest,
      public ::testing::WithParamInterface<rclcpp::Parameter> {};

TEST_P(InvalidParameterTest, ConfigureFails) {
  ASSERT_EQ(init(withParameter(validParameters(), GetParam())),
            controller_interface::return_type::OK);
  EXPECT_FALSE(configure());
}

INSTANTIATE_TEST_SUITE_P(
    MobileRobotControllerTest, InvalidParameterTest,
    ::testing::Values(
        rclcpp::Parameter("left_wheel_joint_name", ""),
        rclcpp::Parameter("right_wheel_joint_name", ""),
        rclcpp::Parameter("wheel_separation", 0.0),
        rclcpp::Parameter("wheel_separation", -0.191),
        rclcpp::Parameter("wheel_radius", 0.0),
        rclcpp::Parameter("wheel_radius", -0.0335),
        rclcpp::Parameter("position_variance_per_meter", 0.0),
        rclcpp::Parameter("position_variance_per_meter", -0.0025),
        rclcpp::Parameter("yaw_variance_per_radian", 0.0),
        rclcpp::Parameter("yaw_variance_per_radian", -0.01),
        rclcpp::Parameter("linear_velocity_variance", 0.0),
        rclcpp::Parameter("linear_velocity_variance", -0.0003),
        rclcpp::Parameter("angular_velocity_variance", 0.0),
        rclcpp::Parameter("angular_velocity_variance", -0.08)),
    [](const ::testing::TestParamInfo<rclcpp::Parameter> &info) {
      std::string name = info.param.get_name();
      if (info.param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
        return name + "_empty";
      }
      return name + (info.param.as_double() < 0.0 ? "_negative" : "_zero");
    });

// ---------------------------------------------------------------------------
// interface configuration
// ---------------------------------------------------------------------------

TEST_F(MobileRobotControllerTest, ClaimsLeftAndRightWheelVelocityCommands) {
  ASSERT_EQ(init(), controller_interface::return_type::OK);
  ASSERT_TRUE(configure());

  const auto config = controller_->command_interface_configuration();
  EXPECT_EQ(config.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  // order matters: update() indexes the interfaces as [left, right]
  EXPECT_THAT(config.names, ElementsAre("left_wheel_joint/velocity",
                                        "right_wheel_joint/velocity"));
}

TEST_F(MobileRobotControllerTest, ClaimsLeftAndRightWheelPositionStates) {
  ASSERT_EQ(init(), controller_interface::return_type::OK);
  ASSERT_TRUE(configure());

  const auto config = controller_->state_interface_configuration();
  EXPECT_EQ(config.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(config.names, ElementsAre("left_wheel_joint/position",
                                        "right_wheel_joint/position"));
}

// ---------------------------------------------------------------------------
// cmd_vel -> wheel commands
// ---------------------------------------------------------------------------

TEST_F(MobileRobotControllerTest, NoCommandYetGivesZeroWheelCommands) {
  ASSERT_NO_FATAL_FAILURE(startController());
  update(0.0, 0.01);
  EXPECT_NEAR(leftCommand(), 0.0, k_eps);
  EXPECT_NEAR(rightCommand(), 0.0, k_eps);
}

TEST_F(MobileRobotControllerTest, CmdVelIsConvertedToWheelCommands) {
  ASSERT_NO_FATAL_FAILURE(startController());
  // v = 0.2 m/s, w = 1 rad/s: left = (0.2 - 0.1) / 0.05, right = (0.2 + 0.1) / 0.05
  sendCommand(0.2, 1.0, 2.0, 6.0);
}

TEST_F(MobileRobotControllerTest, IgnoresNonPlanarTwistComponents) {
  ASSERT_NO_FATAL_FAILURE(startController());
  // a diff drive can only follow linear.x and angular.z; the rest of the
  // twist must not leak into the wheel commands
  auto pub = testNode_->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10);
  ASSERT_TRUE(waitFor([&] { return pub->get_subscription_count() > 0; }));
  geometry_msgs::msg::TwistStamped msg;
  msg.twist.linear.x = -0.1;
  msg.twist.linear.y = 5.0;
  msg.twist.linear.z = 5.0;
  msg.twist.angular.x = 5.0;
  msg.twist.angular.y = 5.0;
  pub->publish(msg);

  ASSERT_TRUE(waitFor([&] {
    executor_->spin_some();
    if (controller_->update(stamp(0.0), period(0.0)) !=
        controller_interface::return_type::OK) {
      return false;
    }
    return std::abs(leftCommand() + 2.0) < k_eps &&
           std::abs(rightCommand() + 2.0) < k_eps;
  })) << "last: " << leftCommand() << ", " << rightCommand();
}

TEST_F(MobileRobotControllerTest, KeepsLastCommandUntilANewOneArrives) {
  // the controller has no command timeout of its own - twist_mux's topic
  // timeouts stop the robot when the joystick goes silent
  ASSERT_NO_FATAL_FAILURE(startController());
  sendCommand(0.2, 0.0, 4.0, 4.0);
  for (int i = 1; i <= 5; ++i) {
    update(0.01 * i, 0.01);
  }
  EXPECT_NEAR(leftCommand(), 4.0, k_eps);
  EXPECT_NEAR(rightCommand(), 4.0, k_eps);
}

// ---------------------------------------------------------------------------
// activate / deactivate
// ---------------------------------------------------------------------------

TEST_F(MobileRobotControllerTest, DeactivateWritesZeroWheelCommands) {
  ASSERT_NO_FATAL_FAILURE(startController());
  sendCommand(0.2, 0.0, 4.0, 4.0);

  ASSERT_TRUE(deactivate());
  EXPECT_NEAR(leftCommand(), 0.0, k_eps);
  EXPECT_NEAR(rightCommand(), 0.0, k_eps);
}

TEST_F(MobileRobotControllerTest, ActivateDropsCommandFromBeforeDeactivation) {
  // otherwise re-activating the controller would make the robot drive off
  // with a command that may be minutes old
  ASSERT_NO_FATAL_FAILURE(startController());
  sendCommand(0.2, 0.0, 4.0, 4.0);
  ASSERT_TRUE(deactivate());

  ASSERT_TRUE(activate());
  update(1.0, 0.01);
  EXPECT_NEAR(leftCommand(), 0.0, k_eps);
  EXPECT_NEAR(rightCommand(), 0.0, k_eps);
}

// ---------------------------------------------------------------------------
// odometry: /odom and /tf
// ---------------------------------------------------------------------------

TEST_F(MobileRobotControllerTest, FirstUpdateCausesNoOdometryJump) {
  // the encoders are not at zero when the controller starts (they count
  // since the ESP32 booted); the first sample is only a reference
  setWheelPositions(5.0, 7.0);
  subscribeOdometry();
  ASSERT_NO_FATAL_FAILURE(startController());

  update(1.0, 0.01);
  publishAndReceive(1.0);
  expectPose(0.0, 0.0, 0.0);
  expectTwist(0.0, 0.0);
}

TEST_F(MobileRobotControllerTest, OdometryFollowsWheelPositions) {
  setWheelPositions(5.0, 7.0);
  subscribeOdometry();
  ASSERT_NO_FATAL_FAILURE(startController());
  update(0.0, 0.01);

  // 0.2 m straight ahead in 0.1 s (4 rad * 0.05 m)
  setWheelPositions(9.0, 11.0);
  update(0.1, 0.1);
  publishAndReceive(0.1);
  expectPose(0.2, 0.0, 0.0);
  expectTwist(2.0, 0.0);

  // quarter turn left on the spot: dHeading = r * (dR - dL) / separation
  setWheelPositions(9.0 - k_pi, 11.0 + k_pi);
  update(0.2, 0.1);
  publishAndReceive(0.2);
  expectPose(0.2, 0.0, k_pi / 2.0);
  expectTwist(0.0, (k_pi / 2.0) / 0.1);

  // 0.2 m straight ahead, now along +y
  setWheelPositions(13.0 - k_pi, 15.0 + k_pi);
  update(0.3, 0.1);
  publishAndReceive(0.3);
  expectPose(0.2, 0.2, k_pi / 2.0);
  expectTwist(2.0, 0.0);
}

TEST_F(MobileRobotControllerTest, OdometryUsesOdomAndBaseLinkFrames) {
  subscribeOdometry();
  ASSERT_NO_FATAL_FAILURE(startController());
  update(1.0, 0.01);
  publishAndReceive(1.0);

  EXPECT_EQ(lastOdom_->header.frame_id, "odom");
  EXPECT_EQ(lastOdom_->child_frame_id, "base_link");
  ASSERT_EQ(lastTf_->transforms.size(), 1u);
  EXPECT_EQ(lastTf_->transforms[0].header.frame_id, "odom");
  EXPECT_EQ(lastTf_->transforms[0].child_frame_id, "base_link");
}

TEST_F(MobileRobotControllerTest, OdometryCovarianceGrowsWithDistanceAndRotation) {
  subscribeOdometry();
  ASSERT_NO_FATAL_FAILURE(startController());
  update(0.0, 0.01);

  // 0.2 m forward, then a quarter turn
  setWheelPositions(4.0, 4.0);
  update(0.1, 0.1);
  setWheelPositions(4.0 - k_pi, 4.0 + k_pi);
  update(0.2, 0.1);
  publishAndReceive(0.2);

  // row-major 6x6, diagonal at index 7 * i
  const auto &pose = lastOdom_->pose.covariance;
  EXPECT_NEAR(pose[0], 0.2 * k_positionVariancePerMeter, k_eps);
  EXPECT_NEAR(pose[7], 0.2 * k_positionVariancePerMeter, k_eps);
  EXPECT_NEAR(pose[14], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(pose[21], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(pose[28], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(pose[35], k_pi / 2.0 * k_yawVariancePerRadian, k_eps);

  // twist variance is a fixed sensor property, it doesn't grow
  const auto &twist = lastOdom_->twist.covariance;
  EXPECT_NEAR(twist[0], k_linearVelocityVariance, k_eps);
  EXPECT_NEAR(twist[7], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(twist[14], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(twist[21], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(twist[28], k_negligibleVariance, 1e-15);
  EXPECT_NEAR(twist[35], k_angularVelocityVariance, k_eps);

  for (std::size_t i = 0; i < 36; ++i) {
    if (i % 7 != 0) {
      EXPECT_NEAR(pose[i], 0.0, 1e-15) << "pose covariance index " << i;
      EXPECT_NEAR(twist[i], 0.0, 1e-15) << "twist covariance index " << i;
    }
  }
}

TEST_F(MobileRobotControllerTest, ZeroPeriodMovesPoseButKeepsLastTwist) {
  // the controller_manager can hand over period == 0 (e.g. first cycle or a
  // clock jump); the twist must not become inf/NaN from dividing by it
  subscribeOdometry();
  ASSERT_NO_FATAL_FAILURE(startController());
  update(0.0, 0.01);
  setWheelPositions(4.0, 4.0);
  update(0.1, 0.1);

  setWheelPositions(8.0, 8.0);
  update(0.2, 0.0);
  publishAndReceive(0.2);
  expectPose(0.4, 0.0, 0.0);
  expectTwist(2.0, 0.0);
}

TEST_F(MobileRobotControllerTest, ReactivationIgnoresWheelMotionWhileInactive) {
  // the robot may be pushed around while the controller is inactive; the
  // pose is kept and that motion must not show up as one big jump
  subscribeOdometry();
  ASSERT_NO_FATAL_FAILURE(startController());
  update(0.0, 0.01);
  setWheelPositions(4.0, 4.0);
  update(0.1, 0.1);
  ASSERT_TRUE(deactivate());

  setWheelPositions(100.0, 50.0);
  ASSERT_TRUE(activate());
  update(1.0, 0.01);
  publishAndReceive(1.0);
  expectPose(0.2, 0.0, 0.0);
  // no stale twist from before the deactivation either
  expectTwist(0.0, 0.0);

  // and it tracks motion again from the new reference
  setWheelPositions(104.0, 54.0);
  update(1.1, 0.1);
  publishAndReceive(1.1);
  expectPose(0.4, 0.0, 0.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
