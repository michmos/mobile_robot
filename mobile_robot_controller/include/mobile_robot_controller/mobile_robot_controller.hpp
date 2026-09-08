#ifndef MOBILE_ROBOT_CONTROLLER_HPP
#define MOBILE_ROBOT_CONTROLLER_HPP

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace mobile_robot_controller {

class MobileRobotController : public controller_interface::ControllerInterface {
private:
  const std::vector<std::string> expected_joints_ = {"left_wheel_joint",
                                                      "right_wheel_joint"};

  // kinematic parameters, read from ROS params in on_configure()
  double wheelSeparation_ = 0.0;
  double wheelRadius_ = 0.0;

  // indices into command_interfaces_/state_interfaces_ (base class members),
  // resolved by name in on_configure(); command_interfaces_.size()/
  // state_interfaces_.size() means "not found"
  size_t leftWheelCommandIdx_ = 0;
  size_t rightWheelCommandIdx_ = 0;
  size_t leftWheelPositionIdx_ = 0;
  size_t rightWheelPositionIdx_ = 0;

  // dead-reckoned pose, integrated in updateOdometry_()
  double x_ = 0.0;
  double y_ = 0.0;
  double heading_ = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
  // written by cmdVelCallback_() (subscription thread), read by update()
  // (realtime thread)
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>>
      cmdVelBuffer_;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odomPub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
      realtimeOdomPub_;

  std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tfPub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
      realtimeTfPub_;

public:
  MobileRobotController() = default;

  // declare joint/wheel_separation/wheel_radius parameters
  controller_interface::CallbackReturn on_init() override;

  // which command interfaces this controller claims (left/right wheel
  // velocity)
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  // which state interfaces this controller claims (left/right wheel
  // position)
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  // lifecycle: INACTIVE -> INACTIVE (also reached from UNCONFIGURED)
  // - read kinematic parameters
  // - resolve interface indices
  // - set up cmd_vel subscription and odom/tf publishers
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: INACTIVE -> ACTIVE
  // - reset pose and command buffer to a known state
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: ACTIVE -> INACTIVE
  // - write a zero velocity command so the robot doesn't keep moving
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // realtime loop, called by the controller manager
  // - read wheel position state interfaces, update odometry
  // - read last cmd_vel, write wheel velocity command interfaces
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // subscription callback, stores msg into cmdVelBuffer_ for update() to pick
  // up
  void cmdVelCallback_(const std::shared_ptr<geometry_msgs::msg::Twist> msg);

  // inverse kinematics: body twist -> {leftWheelVel, rightWheelVel} (rad/s)
  std::pair<double, double> inverseKinematics_(double linearVel,
                                               double angularVel) const;

  // forward kinematics: integrates x_/y_/heading_ from the wheel position
  // delta since the last call
  void updateOdometry_(double leftWheelPos, double rightWheelPos,
                       const rclcpp::Duration &period);

  // publishes the current x_/y_/heading_ as nav_msgs/Odometry and the
  // odom -> base_link tf transform
  void publishOdometry_(const rclcpp::Time &time);
};

} // namespace mobile_robot_controller

#endif
