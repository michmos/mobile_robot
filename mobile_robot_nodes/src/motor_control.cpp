#include "std_msgs/msg/int32.hpp"
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int32.h>

uint16_t getWallDistance(uint32_t data) { return data; }

class MotorControl : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;

public:
  MotorControl() : Node("motor_controler") {

    auto ultraSonicData_subCallback =
        [this](std_msgs::msg::Int32::UniquePtr msg) {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "received the following data: " << msg->data);

          // esp32 has pwm resolution of 16, so we should make use of that

          uint16_t wallDistance = getWallDistance(msg->data);
          uint16_t motorDutyCycle = 0;
          if (wallDistance < 15) {
            // slow down motor and eventually stop
            // TODO: insert slow down routing that doesn't stop the motor
            // instantly
            motorDutyCycle = 0;

          } else {
            // full speed
            motorDutyCycle = 65535;
          }

          // publish message
          // TODO: change publisher message type to something with: value,
          // maxValue
          std_msgs::msg::Int32 newMessage;
          newMessage.data = motorDutyCycle;
          pub_->publish(std::make_unique<std_msgs::msg::Int32>(newMessage));
        };

    sub_ = rclcpp::create_subscription<std_msgs::msg::Int32>(
        *this, "ultra_sonic_data", 10, ultraSonicData_subCallback);

    pub_ = rclcpp::create_publisher<std_msgs::msg::Int32>(*this, "pwm_control",
                                                          10);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  rclcpp::shutdown();

  return (0);
}
