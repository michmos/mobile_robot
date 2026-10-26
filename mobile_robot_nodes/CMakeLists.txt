cmake_minimum_required(VERSION 3.20)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# exits when a hardware component leaves 'active', see
# mobile_robot/hardware_watchdog.py
install(PROGRAMS
  scripts/hardware_watchdog
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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

  find_package(ament_cmake_pytest REQUIRED)

  # APPEND_ENV lets the tests import the `mobile_robot` module from the
  # source tree when the install space isn't sourced.
  ament_add_pytest_test(test_urdf_kinematics test/test_urdf_kinematics.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  # Static checks of description, configs, launch files and plugin.xml.
  # Nothing is started, the launch files are only imported.
  ament_add_pytest_test(test_smoke test/test_smoke.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_hardware_watchdog test/test_hardware_watchdog.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_robot_description test/test_robot_description.py)
  # Only builds the launch description, starts no node.
  ament_add_pytest_test(test_mobile_robot_launch
    test/test_mobile_robot_launch.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )

  # Full bringup with mock hardware. The isolated runner (ament_cmake_ros)
  # picks a free ROS_DOMAIN_ID, and discovery stays on localhost, so test
  # commands never reach a real robot on the LAN. ENV, not APPEND_ENV: the
  # variable takes a single value, appending to an inherited one (the ROS
  # docker images set SUBNET) gives the invalid "SUBNET:LOCALHOST".
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_bringup.launch.py
    TARGET test_bringup
    TIMEOUT 60
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
  # Runs `ros2 launch` as a child process and checks that it exits on its
  # own once the hardware component is deactivated (what systemd sees).
  add_launch_test(test/test_hardware_watchdog.launch.py
    TARGET test_hardware_watchdog_launch
    TIMEOUT 90
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  )
endif()

ament_package()
