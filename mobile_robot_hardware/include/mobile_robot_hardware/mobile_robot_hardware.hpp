#ifndef MOBILE_ROBOT_HARDWARE_HPP
#define MOBILE_ROBOT_HARDWARE_HPP

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/state.hpp>

// shared with the firmware, the include path points at esp32_firmware/
#include "inc/ConfigPayload.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace mobile_robot_hardware {

class MobileRobotHardware : public hardware_interface::SystemInterface {
private:
  const std::vector<std::string> expected_joints_ = {"left_wheel_joint",
                                                     "right_wheel_joint"};

  // <param> tags of the <ros2_control> block, parsed in on_init(). These two
  // stay on the pi, the fields keep the snake_case of the shared struct
  std::string serialPort_;
  uint32_t baudRate_ = 0;

  // forwarded to the esp32 in the Config ("C,...") message
  protocol::ConfigPayload config_;

public:
  MobileRobotHardware() = default;

  // parse <ros2_control> URDF tag, validate joint/interface config
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
      override;

  // lifecycle: UNCONFIGURED -> INACTIVE (open serial port)
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: INACTIVE -> UNCONFIGURED (close serial port)
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: INACTIVE -> ACTIVE (enable motors, start reading real data)
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: ACTIVE -> INACTIVE (disable motors, stop commanding)
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: any -> FINALIZED (final teardown before destruction)
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: any -> UNCONFIGURED via error transition
  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  // realtime loop, called by the controller manager
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type
  write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  //
  // Handles into the framework-owned interfaces, looked up once by name in
  // on_configure() via
  // get_state_interface_handle()/get_command_interface_handle(). The name-based
  // set_state()/get_command() overloads do a map lookup and throw, so
  // read()/write() use these handles instead.
  hardware_interface::StateInterface::SharedPtr leftPositionState_;
  hardware_interface::StateInterface::SharedPtr leftVelocityState_;
  hardware_interface::StateInterface::SharedPtr rightPositionState_;
  hardware_interface::StateInterface::SharedPtr rightVelocityState_;

  hardware_interface::CommandInterface::SharedPtr leftVelocityCommand_;
  hardware_interface::CommandInterface::SharedPtr rightVelocityCommand_;
};

} // namespace mobile_robot_hardware

#endif
