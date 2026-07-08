#include "std_msgs/msg/u_int16.hpp"
#include <cstdint>
#include <memory>
#include <ratio>
#include <rclcpp/rclcpp.hpp>

constexpr double kSpeedOfSound = 0.034; // in cm/us

using UInt16 = std_msgs::msg::UInt16;

class Perception : public rclcpp::Node {
private:
  rclcpp::Subscription<UInt16>::SharedPtr sub_;
  rclcpp::Publisher<UInt16>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint16_t> distance_;

public:
  Perception() : Node("perception") {
    auto subCallback = [this](std::unique_ptr<UInt16> msg) {
      RCLCPP_DEBUG(this->get_logger(), "perception read from '%s': %d",
                   "ultrasonic_raw", msg->data);

      const uint16_t duration = msg->data;
      distance_.store((duration * kSpeedOfSound) / 2);
    };

    auto timerCallback = [this]() {
      UInt16 newMessage;
      newMessage.data = distance_.load();
      pub_->publish(std::move(newMessage));
    };

    sub_ = this->create_subscription<UInt16>("ultrasonic_raw", 10, subCallback);
    pub_ = this->create_publisher<UInt16>("distance", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), timerCallback);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Perception>());
  rclcpp::shutdown();
  return 0;
}
