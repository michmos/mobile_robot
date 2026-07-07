#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/int32.h>

uint16_t getWallDistance(uint32_t data) { return data; }


class MotorControl : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  // TODO: change publisher message type to something with: value,
  // maxValue, direction and that for both wheels
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  uint32_t currentDutyCycle_;
  bool inSpeedTransition_;

public:
  MotorControl()
      : Node("motor_controler"), currentDutyCycle_(0),
        inSpeedTransition_(false) {

    auto ultraSonicData_subCallback =
        [this](std_msgs::msg::Int32::UniquePtr msg) {
          RCLCPP_INFO_STREAM(this->get_logger(),
                             "received distance: " << msg->data);

          // no adaptions to speed during speed transitions
          if (inSpeedTransition_) {
            return;
          }

          uint16_t wallDistance = getWallDistance(msg->data);
          if (wallDistance < 15) {
            transitionSpeedTo(0);

          } else {
            transitionSpeedTo(65535);
          }

          // publish message
          std_msgs::msg::Int32 newMessage;
          newMessage.data = currentDutyCycle_;
          pub_->publish(std::make_unique<std_msgs::msg::Int32>(newMessage));
        };

    sub_ = rclcpp::create_subscription<std_msgs::msg::Int32>(
        *this, "ultra_sonic_data", 10, ultraSonicData_subCallback);

    pub_ = rclcpp::create_publisher<std_msgs::msg::Int32>(*this, "pwm_control",
                                                          10);
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
    std_msgs::msg::Int32 msg;
    msg.data = newSpeed;
    pub_->publish(std::make_unique<std_msgs::msg::Int32>(std::move(msg)));
    RCLCPP_INFO_STREAM(this->get_logger(), "publishing speed: " << msg.data);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  rclcpp::shutdown();

  return (0);
}
