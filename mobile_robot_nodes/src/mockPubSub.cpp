#include "std_msgs/msg/int32.hpp"
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/int32.h>

class MotorControl : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;

public:
  MotorControl() : Node("motor_controler") {
    auto subCallback = [this](std_msgs::msg::Int32::UniquePtr msg) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "received the following data: " << msg->data);

      // simulate some motor control maths
      // take the adc value / divide it by and multiply by 100 to get a
      // percentage that percentage we publish to control the motors
      uint16_t adcVal = msg->data;
      uint8_t percentage = ((long)adcVal * 100) / 4095;

      // publish message
      std_msgs::msg::Int32 newMessage;
      newMessage.data = percentage;
      pub_->publish(std::make_unique<std_msgs::msg::Int32>(newMessage));
    };

    sub_ = rclcpp::create_subscription<std_msgs::msg::Int32>(*this, "adc_value",
                                                             10, subCallback);

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
