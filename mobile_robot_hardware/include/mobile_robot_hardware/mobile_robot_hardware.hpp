#ifndef MOBILE_ROBOT_HARDWARE_HPP
#define MOBILE_ROBOT_HARDWARE_HPP

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <io_context/io_context.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <serial_driver/serial_driver.hpp>

// shared with the firmware, the include path points at esp32_firmware/
#include "inc/ConfigPayload.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace mobile_robot_hardware {

class MobileRobotHardware : public hardware_interface::SystemInterface {
private:
  const std::vector<std::string> expected_joints_ = {"left_wheel_joint",
                                                     "right_wheel_joint"};

  // ioContext must outlive driver: SerialDriver only stores a reference to it
  struct Serial {
    drivers::common::IoContext ioContext;
    drivers::serial_driver::SerialDriver driver;
    std::shared_ptr<drivers::serial_driver::SerialPort>
        port; // null until opened

    // fed by the async_receive callback in on_configure(); readLine_() drains
    // it
    std::string rxBuffer;
    std::mutex rxMutex;
    std::condition_variable rxCv;

    Serial() : driver(ioContext) {}
  } serial_;

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

  // lifecycle: UNCONFIGURED -> INACTIVE
  // - validate and cash interface handles
  // - open serial port
  // - perform handshake
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: INACTIVE -> UNCONFIGURED (close serial port)
  // - close serial
  // - clear serial buffer
  // - release interface handles
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: INACTIVE -> ACTIVE
  // - init command interfaces to safe values
  // - clear serial buffer
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: ACTIVE -> INACTIVE
  // - send stop command
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: any -> FINALIZED (final teardown; reuses on_deactivate()/
  // on_cleanup() since this can be reached directly from any state)
  // - call on_deactivate() if previously active
  // - call on_cleanup() if previously active or inactive
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

  // lifecycle: any -> UNCONFIGURED via error transition
  // - call on_deactivate() if previously active
  // - call on_cleanup()
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

  // used to compute HW_IF_VELOCITY from successive tick deltas; seeded by
  // on_activate()
  int32_t lastLeftTicks_ = 0;
  int32_t lastRightTicks_ = 0;
  uint32_t lastSampleTimeUs_ = 0;
  // false until read() has consumed a first sample after on_activate()
  bool hasEncoderBaseline_ = false;

  // blocks up to `timeout` for a '\n'-terminated line in serial_.rxBuffer
  // @param line: initialized with retrieved line
  // @param timeout: time to wait for '\n'
  bool readLine_(std::string &line, std::chrono::milliseconds timeout);

  // esp32 handshake: wait for its SETUP event, send config_, wait for ACK/NACK
  // @throws std::runtime_error naming which step failed
  void performHandshake_(std::chrono::milliseconds setupTimeout,
                         std::chrono::milliseconds ackTimeout);

  // serialize and send an "M,..." velocity command to the esp32
  // @throws std::runtime_error / asio exceptions on a send failure
  void sendMotorCmd_(float leftVelCmd, float rightVelCmd);
};

} // namespace mobile_robot_hardware

#endif
