cmake_minimum_required(VERSION 3.20)
project(mobile_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/mobile_robot_controller.cpp
  src/kinematics.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${controller_interface_TARGETS}
  ${hardware_interface_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  tf2::tf2
  ${pluginlib_TARGETS}
  ${realtime_tools_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
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

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  realtime_tools
  geometry_msgs
  nav_msgs
  tf2_msgs
)

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

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics ${PROJECT_NAME})

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  # these tests create ROS nodes and publish on /cmd_vel: the isolated variant
  # gives each test its own ROS domain limited to localhost, so a robot on the
  # same network can't receive the commands and outside traffic can't reach
  # the tests
  ament_add_ros_isolated_gmock(test_mobile_robot_controller test/test_mobile_robot_controller.cpp)
  target_link_libraries(test_mobile_robot_controller
    ${PROJECT_NAME}
    ${lifecycle_msgs_TARGETS}
  )

  # loads the plugin through pluginlib, so it needs the installed package
  # (source the install space before running it)
  ament_add_ros_isolated_gmock(test_load_mobile_robot_controller test/test_load_mobile_robot_controller.cpp)
  target_link_libraries(test_load_mobile_robot_controller
    controller_manager::controller_manager
  )
endif()

ament_package()
