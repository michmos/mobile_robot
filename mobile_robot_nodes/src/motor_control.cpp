#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/u_int16.hpp>

using UInt16 = std_msgs::msg::UInt16;
using Int32 = std_msgs::msg::Int32;

class MotorControl : public rclcpp::Node {
private:
  rclcpp::Subscription<UInt16>::SharedPtr sub_;
  // TODO: change publisher message type to something with: value,
  // maxValue, direction and that for both wheels
  rclcpp::Publisher<Int32>::SharedPtr pub_;
  uint32_t currentDutyCycle_;
  bool inSpeedTransition_;

public:
  MotorControl()
      : Node("motor_controler"), currentDutyCycle_(0),
        inSpeedTransition_(false) {

    auto subCallback = [this](UInt16::UniquePtr msg) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "received distance: " << msg->data);

      // no adaptions to speed during speed transitions
      if (inSpeedTransition_) {
        return;
      }

      uint16_t wallDistance = msg->data;
      if (wallDistance < 15) {
        transitionSpeedTo(0);

      } else {
        transitionSpeedTo(65535);
      }

      // publish message
      Int32 newMessage;
      newMessage.data = currentDutyCycle_;
      pub_->publish(std::make_unique<Int32>(newMessage));
    };

    sub_ =
        rclcpp::create_subscription<UInt16>(*this, "distance", 10, subCallback);

    pub_ = rclcpp::create_publisher<Int32>(*this, "pwm_control", 10);
  }

  // TODO: probably shouldn't sleep here
  // transitions to new speed value
  void transitionSpeedTo(uint32_t newSpeed = 0) {
    if (currentDutyCycle_ == newSpeed) {
      return;
    } else if (newSpeed > 65535) {
      RCLCPP_DEBUG_STREAM(
          this->get_logger(),
          "changeSpeed() called with invalid value: " << newSpeed);
      return;
    }

    // TODO: change these parameters to tune the transition
    const std::chrono::milliseconds transitionTime(1000);
    const uint8_t steps = 10;

    std::chrono::milliseconds stepTime(transitionTime / steps);

    inSpeedTransition_ = true;

    uint32_t startDutyCycle = currentDutyCycle_;
    for (int i = 1; i <= steps; ++i) {
      changeSpeedTo(startDutyCycle + (newSpeed - startDutyCycle) * i / steps);
      rclcpp::sleep_for(stepTime);
    }
    currentDutyCycle_ = newSpeed;

    inSpeedTransition_ = false;
  }

  // instantly changes to new speed value
  void changeSpeedTo(uint32_t newSpeed) {
    if (currentDutyCycle_ == newSpeed) {
      return;
    } else if (newSpeed > 65535) {
      RCLCPP_DEBUG_STREAM(
          this->get_logger(),
          "changeSpeed() called with invalid value: " << newSpeed);
      return;
    }

    currentDutyCycle_ = newSpeed;
    Int32 msg;
    msg.data = newSpeed;
    pub_->publish(std::make_unique<Int32>(std::move(msg)));
    RCLCPP_INFO_STREAM(this->get_logger(), "publishing speed: " << msg.data);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  rclcpp::shutdown();

  return (0);
}
