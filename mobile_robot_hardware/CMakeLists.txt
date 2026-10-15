cmake_minimum_required(VERSION 3.20)
project(mobile_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(io_context REQUIRED)
find_package(serial_driver REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mobile_robot_hardware.cpp
  src/params.cpp
  src/wire_protocol.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# inc/ConfigPayload.hpp defines the payload of the Config message and its range
# check, and is shared with the firmware. It has to live in the sketch folder
# because the arduino build cannot include headers from outside it, so the
# plugin reaches over instead of the other way around
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${hardware_interface_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${pluginlib_TARGETS}
  ${io_context_LIBRARIES}
  ${serial_driver_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${io_context_INCLUDE_DIRS}
  ${serial_driver_INCLUDE_DIRS}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# ConfigPayload.hpp is only on the BUILD_INTERFACE include path (it lives in
# esp32_firmware/, shared with the firmware build), so it has to be installed
# alongside for the exported target to be usable from another package
install(
  FILES ../esp32_firmware/inc/ConfigPayload.hpp
  DESTINATION include/inc
)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface rclcpp rclcpp_lifecycle pluginlib io_context serial_driver)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../lint/lint.cmake)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_handshake test/test_handshake.cpp)
  target_include_directories(test_handshake PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_wire_protocol
    test/test_wire_protocol.cpp
    src/wire_protocol.cpp
  )
  # the round trip tests parse with the firmware's own msgs::Config, which
  # only needs millis() from the Arduino core
  target_include_directories(test_wire_protocol PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  # pure functions, no node and no serial port: only the hardware_interface
  # types and lexical casts are needed, not the whole plugin
  ament_add_gtest(test_params
    test/test_params.cpp
    src/params.cpp
  )
  target_include_directories(test_params PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
  )
  target_link_libraries(test_params ${hardware_interface_TARGETS})

  # drives the plugin through its lifecycle against a fake esp32 on a pty
  # (test/fake_esp32.hpp). The failing handshake cases wait for their
  # (shortened) timeout, the whole file takes a few seconds
  ament_add_gtest(test_mobile_robot_hardware
    test/test_mobile_robot_hardware.cpp
    TIMEOUT 60
    # the plugin creates a node, keep its DDS discovery on this machine
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
  target_include_directories(test_mobile_robot_hardware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )
  # openpty() lives in libutil
  target_link_libraries(test_mobile_robot_hardware ${PROJECT_NAME} util)

  # encoder reports and the other incoming lines turned into joint states,
  # same fake esp32 as above. The plugin creates a node: the isolated variant
  # runs the test on its own ROS domain, kept on this machine
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_encoder_state
    test/test_encoder_state.cpp
    TIMEOUT 60
  )
  target_include_directories(test_encoder_state PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )
  target_link_libraries(test_encoder_state ${PROJECT_NAME} util)

  # host tests for the hardware independent firmware code in esp32_firmware/inc/,
  # the Arduino core is replaced by the controllable fake in test/arduino_stub
  ament_add_gtest(test_arduino_stub test/firmware/test_arduino_stub.cpp)
  target_include_directories(test_arduino_stub PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_motor_control test/firmware/test_motor_control.cpp)
  target_include_directories(test_motor_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_scheduler test/firmware/test_scheduler.cpp)
  target_include_directories(test_scheduler PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_velocity test/firmware/test_velocity.cpp)
  target_include_directories(test_velocity PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_drive test/firmware/test_drive.cpp)
  target_include_directories(test_drive PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_l298 test/firmware/test_l298.cpp)
  target_include_directories(test_l298 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_encoder test/firmware/test_encoder.cpp)
  target_include_directories(test_encoder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_serial_channel test/firmware/test_serial_channel.cpp)
  target_include_directories(test_serial_channel PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  ament_add_gtest(test_messages test/firmware/test_messages.cpp)
  target_include_directories(test_messages PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )

  # pins the parts of the wire format both sides have to agree on, also
  # against the firmware's msgs::* serializers and parsers
  ament_add_gtest(test_wire_contract
    test/test_wire_contract.cpp
    src/wire_protocol.cpp
  )
  target_include_directories(test_wire_contract PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../esp32_firmware
    ${CMAKE_CURRENT_SOURCE_DIR}/test/arduino_stub
  )
endif()

ament_package()
