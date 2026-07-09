#include <chrono>
#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <limits>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/int16.hpp>

constexpr double kWheelDiameter = 0.065; // in m
constexpr double kPerimeter = kWheelDiameter * M_PI;
constexpr double kMaxRpm = 200;

using Int16 = std_msgs::msg::Int16;
using Twist = geometry_msgs::msg::Twist;

class MotorControl : public rclcpp::Node {
private:
  rclcpp::Subscription<Twist>::SharedPtr sub_;
  rclcpp::Publisher<Int16>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<int16_t> dutyCycle_;

public:
  MotorControl() : Node("motor_controller"), dutyCycle_(0) {

    auto subCallback = [this](Twist::UniquePtr msg) {
      double rpm = msg->linear.x / kPerimeter * 60;
      if (rpm > kMaxRpm || rpm < -kMaxRpm) {
        RCLCPP_WARN(this->get_logger(),
                    "received velocity (linear.x) higher than possible. Max "
                    "speed will be used instead");
        rpm = kMaxRpm;
      }
      dutyCycle_.store(rpm / kMaxRpm * std::numeric_limits<int16_t>::max());
      // TODO: add rotational stuff
    };

    auto timerCallback = [this]() {
      // publish message
      Int16 newMessage;
      newMessage.data = dutyCycle_.load();
      pub_->publish(std::move(newMessage));
    };

    sub_ = this->create_subscription<Twist>("cmd_vel", 10, subCallback);

    pub_ = this->create_publisher<Int16>("pwm_control", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), timerCallback);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControl>());
  rclcpp::shutdown();

  return (0);
}
