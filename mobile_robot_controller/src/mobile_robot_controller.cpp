#include "mobile_robot_controller/mobile_robot_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

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
  // TODO: implement
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MobileRobotController::on_activate(const rclcpp_lifecycle::State &) {
  // TODO: implement
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
