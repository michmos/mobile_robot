#include "mobile_robot_controller/mobile_robot_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

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
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {leftWheelJointName_ + "/" + hardware_interface::HW_IF_VELOCITY,
           rightWheelJointName_ + "/" + hardware_interface::HW_IF_VELOCITY}};
}

controller_interface::InterfaceConfiguration
MobileRobotController::state_interface_configuration() const {
  // individual = claim only requested interfaces - not all
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          {leftWheelJointName_ + "/" + hardware_interface::HW_IF_POSITION,
           rightWheelJointName_ + "/" + hardware_interface::HW_IF_POSITION}};
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
  // TODO: implement
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
MobileRobotController::update(const rclcpp::Time &, const rclcpp::Duration &) {
  // TODO: implement
  return controller_interface::return_type::OK;
}

void MobileRobotController::cmdVelCallback_(
    const std::shared_ptr<geometry_msgs::msg::Twist>) {
  // TODO: implement
}

std::pair<double, double>
MobileRobotController::inverseKinematics_(double, double) const {
  // TODO: implement
  return {0.0, 0.0};
}

void MobileRobotController::updateOdometry_(double, double,
                                            const rclcpp::Duration &) {
  // TODO: implement
}

void MobileRobotController::publishOdometry_(const rclcpp::Time &) {
  // TODO: implement
}

PLUGINLIB_EXPORT_CLASS(mobile_robot_controller::MobileRobotController,
                       controller_interface::ControllerInterface)
