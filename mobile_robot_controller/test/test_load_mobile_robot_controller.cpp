// Checks that the controller_manager can find and load the plugin by the type
// name used in controller_manager.yaml, i.e. that plugin.xml and the
// pluginlib export match.

#include <gmock/gmock.h>

#include <controller_manager/controller_manager.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace {

// two wheel joints on mock hardware - just enough for the resource manager to
// start, the controller is only loaded, not activated
const std::string k_urdf = R"(<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <axis xyz="0 0 1"/>
  </joint>
  <ros2_control name="test_system" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
)";

} // namespace

TEST(LoadMobileRobotController, ControllerManagerLoadsPlugin) {
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  controller_manager::ControllerManager cm(executor, k_urdf, true,
                                           "test_controller_manager");

  cm.set_parameter({"mobile_robot_controller.type",
                    "mobile_robot_controller/MobileRobotController"});

  const auto controller = cm.load_controller("mobile_robot_controller");
  ASSERT_NE(controller, nullptr);
  EXPECT_EQ(controller->get_node()->get_name(),
            std::string("mobile_robot_controller"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
