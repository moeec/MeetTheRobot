#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class RobotSubscriber : public rclcpp::Node
{
public:
  RobotSubscriber()
  : rclcpp::Node("robot_subscriber")
  {
    auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };

    sub_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, topic_callback);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotSubscriber>());
  rclcpp::shutdown();
  return 0;
}

