cmake_minimum_required(VERSION 3.20)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(control_node src/motor_control.cpp)
target_link_libraries(control_node PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS})

add_executable(perception_node src/perception.cpp)
target_link_libraries(perception_node PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(collision_avoid_node src/collision_avoid.cpp)
target_link_libraries(collision_avoid_node PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS} ${geometry_msgs_TARGETS})

install(TARGETS control_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS perception_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS collision_avoid_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  data
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




ament_package()
