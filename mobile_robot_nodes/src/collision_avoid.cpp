#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

constexpr double kMaxSpeed = 0.25; // m/s

using UInt16 = std_msgs::msg::UInt16;
using Twist = geometry_msgs::msg::Twist;

class CollisionAvoid : public rclcpp::Node {
private:
  rclcpp::Subscription<UInt16>::SharedPtr sub_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  std::atomic<uint16_t> distance_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  CollisionAvoid() : Node("collision_avoid") {
    auto subCallback = [this](UInt16::UniquePtr msg) {
      distance_.store(msg->data);
      RCLCPP_DEBUG(this->get_logger(), "collision_avoid read on 'distance': %d",
                   distance_.load());
    };

    auto timerCallback = [this]() {
      Twist newMessage;
      if (distance_.load() < 15) {
        // TODO: implement turning routine
        newMessage.linear.x = 0;
      } else {
        newMessage.linear.x = kMaxSpeed;
      }
      pub_->publish(newMessage);
    };

    sub_ = this->create_subscription<UInt16>("distance", 10, subCallback);
    pub_ = this->create_publisher<Twist>("cmd_vel", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), timerCallback);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionAvoid>());
  rclcpp::shutdown();
  return 0;
}
