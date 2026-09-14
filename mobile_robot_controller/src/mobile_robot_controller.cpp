#include "mobile_robot_controller/mobile_robot_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
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

  // set up subscriptions and publishers
  cmdVelSub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](std::shared_ptr<geometry_msgs::msg::Twist> msg) {
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

  // set cmd_vel buffer to safe command - Twist's default constructor already
  // zero-initializes linear/angular, no need to set fields individually
  cmdVelBuffer_.initRT(std::make_shared<geometry_msgs::msg::Twist>());

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
  auto t = cmdVelBuffer_.readFromRT();
  auto cmds = inverseKinematics_(t->get()->linear.x, t->get()->angular.z);
  if (!command_interfaces_[LEFT].set_value(cmds.first) ||
      !command_interfaces_[RIGHT].set_value(cmds.second)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "update(): failed to write velocity command");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void MobileRobotController::cmdVelCallback_(
    const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
  cmdVelBuffer_.writeFromNonRT(msg);
}

std::pair<double, double>
MobileRobotController::inverseKinematics_(double, double) const {
  // TODO: implement
  return {0.0, 0.0};
}

void MobileRobotController::updateOdometry_(double leftJointPose,
                                            double rightJointPose,
                                            const rclcpp::Duration &period) {
  if (!lastJointPose_.outdated) {
    double distanceLeft = (leftJointPose - lastJointPose_.left) * wheelRadius_;
    double distanceRight =
        (rightJointPose - lastJointPose_.right) * wheelRadius_;
    double distanceCenter = (distanceLeft + distanceRight) / 2.0;
    double deltaHeading = (distanceRight - distanceLeft) / wheelSeparation_;

    // mid point approx to get new pose
    double headingMid = pose_.heading + deltaHeading / 2.0;
    pose_.x += distanceCenter * cos(headingMid);
    pose_.y += distanceCenter * sin(headingMid);
    pose_.heading += deltaHeading;

    // get velocities - only valid if the period was non-zero
    double dt = period.seconds();
    if (dt > 0.0) {
      twist_.linearVelocity = distanceCenter / dt;
      twist_.angularVelocity = deltaHeading / dt;
    }
  }

  lastJointPose_.outdated = false;
  lastJointPose_.left = leftJointPose;
  lastJointPose_.right = rightJointPose;
}

void MobileRobotController::publishOdometry_(const rclcpp::Time &time) {
  auto odomMsg = nav_msgs::msg::Odometry();

  odomMsg.header.stamp = time;
  odomMsg.header.frame_id = "odom";
  odomMsg.child_frame_id = "base_link";

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_.heading);

  odomMsg.pose.pose.position.x = pose_.x;
  odomMsg.pose.pose.position.y = pose_.y;
  odomMsg.pose.pose.orientation.x = 0.0;
  odomMsg.pose.pose.orientation.y = 0.0;
  odomMsg.pose.pose.orientation.z = q.z();
  odomMsg.pose.pose.orientation.w = q.w();
  // odomMsg.pose.covariance; //TODO: add covariance

  odomMsg.twist.twist.linear.x = twist_.linearVelocity;
  odomMsg.twist.twist.angular.z = twist_.angularVelocity;
  // odomMsg.twist.covariance; // TODO: add covariance

  realtimeOdomPub_->try_publish(odomMsg);

  auto tfMsg = tf2_msgs::msg::TFMessage();
  tfMsg.transforms.resize(1);
  tfMsg.transforms[0].header.stamp = time;
  tfMsg.transforms[0].header.frame_id = "odom";
  tfMsg.transforms[0].child_frame_id = "base_link";
  tfMsg.transforms[0].transform.translation.x = pose_.x;
  tfMsg.transforms[0].transform.translation.y = pose_.y;
  tfMsg.transforms[0].transform.translation.z = 0.0;
  tfMsg.transforms[0].transform.rotation.x = 0.0;
  tfMsg.transforms[0].transform.rotation.y = 0.0;
  tfMsg.transforms[0].transform.rotation.z = q.z();
  tfMsg.transforms[0].transform.rotation.w = q.w();

  realtimeTfPub_->try_publish(tfMsg);
}

PLUGINLIB_EXPORT_CLASS(mobile_robot_controller::MobileRobotController,
                       controller_interface::ControllerInterface)
