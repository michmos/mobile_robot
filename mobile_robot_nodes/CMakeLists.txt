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
  # Linters (ament_lint_auto) are wired up separately in #5.
  find_package(ament_cmake_pytest REQUIRED)

  # APPEND_ENV lets the tests import the `mobile_robot` module from the
  # source tree when the install space isn't sourced.
  ament_add_pytest_test(test_urdf_kinematics test/test_urdf_kinematics.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_robot_description test/test_robot_description.py)
  # Only builds the launch description, starts no node.
  ament_add_pytest_test(test_mobile_robot_launch
    test/test_mobile_robot_launch.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
