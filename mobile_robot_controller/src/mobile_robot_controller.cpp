#include "mobile_robot_controller/mobile_robot_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mobile_robot_controller/kinematics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <controller_interface/controller_interface_base.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>

using namespace mobile_robot_controller;

controller_interface::CallbackReturn MobileRobotController::on_init() {
  // actual values will be read in on_configure, since on_init() can run
  // before parameter overrides from launch file have settled
  try {
    get_node()->declare_parameter<std::string>("left_wheel_joint_name", "");
    get_node()->declare_parameter<std::string>("right_wheel_joint_name", "");
    get_node()->declare_parameter<double>("wheel_separation", 0.0);
    get_node()->declare_parameter<double>("wheel_radius", 0.0);
    get_node()->declare_parameter<double>("position_variance_per_meter", 0.0);
    get_node()->declare_parameter<double>("yaw_variance_per_radian", 0.0);
    get_node()->declare_parameter<double>("linear_velocity_variance", 0.0);
    get_node()->declare_parameter<double>("angular_velocity_variance", 0.0);
    // same default as diff_drive_controller
    get_node()->declare_parameter<double>("cmd_vel_timeout", 0.5);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s",
                 e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MobileRobotController::command_interface_configuration() const {
  // individual = claim only requested interfaces - not all
  std::vector<std::string> interfaceNames(2);
  interfaceNames[LEFT] =
      leftWheelJointName_ + "/" + hardware_interface::HW_IF_VELOCITY;
  interfaceNames[RIGHT] =
      rightWheelJointName_ + "/" + hardware_interface::HW_IF_VELOCITY;
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interfaceNames};
}

controller_interface::InterfaceConfiguration
MobileRobotController::state_interface_configuration() const {
  // individual = claim only requested interfaces - not all
  std::vector<std::string> interfaceNames(2);
  interfaceNames[LEFT] =
      leftWheelJointName_ + "/" + hardware_interface::HW_IF_POSITION;
  interfaceNames[RIGHT] =
      rightWheelJointName_ + "/" + hardware_interface::HW_IF_POSITION;
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          interfaceNames};
}

controller_interface::CallbackReturn
MobileRobotController::on_configure(const rclcpp_lifecycle::State &) {
  // read parameter values
  leftWheelJointName_ =
      get_node()->get_parameter("left_wheel_joint_name").as_string();
  rightWheelJointName_ =
      get_node()->get_parameter("right_wheel_joint_name").as_string();
  wheelSeparation_ = get_node()->get_parameter("wheel_separation").as_double();
  wheelRadius_ = get_node()->get_parameter("wheel_radius").as_double();
  variances_.positionPerMeter =
      get_node()->get_parameter("position_variance_per_meter").as_double();
  variances_.yawPerRadian =
      get_node()->get_parameter("yaw_variance_per_radian").as_double();
  variances_.linearVelocity =
      get_node()->get_parameter("linear_velocity_variance").as_double();
  variances_.angularVelocity =
      get_node()->get_parameter("angular_velocity_variance").as_double();
  cmdVelTimeout_ = get_node()->get_parameter("cmd_vel_timeout").as_double();

  // validate params
  if (leftWheelJointName_.empty() || rightWheelJointName_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "left_wheel_joint_name/right_wheel_joint_name must not be "
                 "empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (wheelSeparation_ <= 0.0 || wheelRadius_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "wheel_separation/wheel_radius must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (variances_.positionPerMeter <= 0.0 || variances_.yawPerRadian <= 0.0 ||
      variances_.linearVelocity <= 0.0 || variances_.angularVelocity <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "position_variance_per_meter/yaw_variance_per_radian/"
                 "linear_velocity_variance/angular_velocity_variance must "
                 "be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }
  // a timeout of 0 or less would either stop the robot on every cycle or
  // (if read as "disabled") let it drive on forever when commands stop
  if (cmdVelTimeout_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "cmd_vel_timeout must be > 0");
    return controller_interface::CallbackReturn::ERROR;
  }

  // set up subscriptions and publishers
  cmdVelSub_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
          "cmd_vel", 10,
          [this](std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
            cmdVelCallback_(msg);
          });
  odomPub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  realtimeOdomPub_ = std::make_unique<
      realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odomPub_);

  // absolute topic: tf must be global, not namespaced under this controller
  tfPub_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
  realtimeTfPub_ = std::make_unique<
      realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(tfPub_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobileRobotController::on_activate(const rclcpp_lifecycle::State &) {
  // leave pose_ untouched to ignore any movement during deactivation

  // mark as outdated, so update() ignores the first position delta
  // for pose updates, ignoring any movement during deactivation
  lastJointPose_.outdated = true;
  // reset twist so that no stale twist is published on reactivation
  twist_ = {};

  // set cmd_vel buffer to safe command - TwistStamped's default constructor
  // zero-initializes linear/angular, no need to set fields individually
  cmdVelBuffer_.initRT(std::make_shared<geometry_msgs::msg::TwistStamped>());
  // forget the command seen before deactivation, the first update() takes
  // the buffer's content as a new command
  lastCmdVel_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobileRobotController::on_deactivate(const rclcpp_lifecycle::State &) {
  if (!command_interfaces_[LEFT].set_value(0.0) ||
      !command_interfaces_[RIGHT].set_value(0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to write zero velocity command on deactivate");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
MobileRobotController::update(const rclcpp::Time &time,
                              const rclcpp::Duration &period) {
  // read states
  auto leftJointPose = state_interfaces_[LEFT].get_optional();
  auto rightJointPose = state_interfaces_[RIGHT].get_optional();
  if (!leftJointPose.has_value() || !rightJointPose.has_value()) {
    RCLCPP_ERROR(get_node()->get_logger(), "update(): failed to read states");
    return controller_interface::return_type::ERROR;
  }

  updateOdometry_(*leftJointPose, *rightJointPose, period);
  publishOdometry_(time);

  // write commands
  const auto &cmdVel = *cmdVelBuffer_.readFromRT();
  if (cmdVel != lastCmdVel_) {
    // first update() that sees this command: take its time as receive time
    lastCmdVel_ = cmdVel;
    lastCmdVelTime_ = time;
  }

  // stop if the velocity source went quiet (e.g. twist_mux timed out its
  // input, which stops forwarding instead of publishing zero). A negative
  // age (time jumped back, e.g. a sim reset) also counts as stale.
  const double cmdVelAge = (time - lastCmdVelTime_).seconds();
  const bool cmdVelStale = cmdVelAge > cmdVelTimeout_ || cmdVelAge < 0.0;
  double linearVel = 0.0;
  double angularVel = 0.0;
  if (!cmdVelStale) {
    linearVel = lastCmdVel_->twist.linear.x;
    angularVel = lastCmdVel_->twist.angular.z;
  }

  auto cmds = inverseKinematics(linearVel, angularVel, wheelSeparation_,
                                wheelRadius_);
  if (!command_interfaces_[LEFT].set_value(cmds.first) ||
      !command_interfaces_[RIGHT].set_value(cmds.second)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "update(): failed to write velocity command");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void MobileRobotController::cmdVelCallback_(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
  cmdVelBuffer_.writeFromNonRT(msg);
}

void MobileRobotController::updateOdometry_(double leftJointPose,
                                            double rightJointPose,
                                            const rclcpp::Duration &period) {
  if (!lastJointPose_.outdated) {
    BodyDelta delta = integrateOdometry(
        pose_, leftJointPose - lastJointPose_.left,
        rightJointPose - lastJointPose_.right, wheelSeparation_, wheelRadius_);

    // increment pose variance by how far the robot actually moved this cycle
    growPoseVariance(poseVariance_, delta, variances_.positionPerMeter,
                     variances_.yawPerRadian);

    // get velocities - only valid if the period was non-zero
    double dt = period.seconds();
    if (dt > 0.0) {
      twist_.linearVelocity = delta.distance / dt;
      twist_.angularVelocity = delta.heading / dt;
    }
  }

  lastJointPose_.outdated = false;
  lastJointPose_.left = leftJointPose;
  lastJointPose_.right = rightJointPose;
}

void MobileRobotController::publishOdometry_(const rclcpp::Time &time) {
  auto odomMsg = nav_msgs::msg::Odometry();

  odomMsg.header.stamp = time;
  odomMsg.header.frame_id = odomFrameId_;
  odomMsg.child_frame_id = baseFrameId_;

  const auto q = yawToQuaternion(pose_.heading);

  odomMsg.pose.pose.position.x = pose_.x;
  odomMsg.pose.pose.position.y = pose_.y;
  odomMsg.pose.pose.orientation = q;

  odomMsg.twist.twist.linear.x = twist_.linearVelocity;
  odomMsg.twist.twist.angular.z = twist_.angularVelocity;

  // "negligible variance" placeholder for the degrees of freedom this
  // planar robot cannot physically move - kept nonzero
  // (rather than exactly 0) since some odometry consumers (e.g.
  // robot_localization) treat an exact 0 variance ambiguously
  constexpr double kNegligibleVariance = 1e-9;

  setDiagonalCovariance(odomMsg.pose.covariance, poseVariance_.position,
                        poseVariance_.position, kNegligibleVariance,
                        kNegligibleVariance, kNegligibleVariance,
                        poseVariance_.yaw);
  setDiagonalCovariance(odomMsg.twist.covariance, variances_.linearVelocity,
                        kNegligibleVariance, kNegligibleVariance,
                        kNegligibleVariance, kNegligibleVariance,
                        variances_.angularVelocity);

  realtimeOdomPub_->try_publish(odomMsg);

  auto tfMsg = tf2_msgs::msg::TFMessage();
  tfMsg.transforms.resize(1);
  tfMsg.transforms[0].header.stamp = time;
  tfMsg.transforms[0].header.frame_id = odomFrameId_;
  tfMsg.transforms[0].child_frame_id = baseFrameId_;
  tfMsg.transforms[0].transform.translation.x = pose_.x;
  tfMsg.transforms[0].transform.translation.y = pose_.y;
  tfMsg.transforms[0].transform.translation.z = 0.0;
  tfMsg.transforms[0].transform.rotation = q;

  realtimeTfPub_->try_publish(tfMsg);
}

PLUGINLIB_EXPORT_CLASS(mobile_robot_controller::MobileRobotController,
                       controller_interface::ControllerInterface)
