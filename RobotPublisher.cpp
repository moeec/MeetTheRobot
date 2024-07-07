#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/*

The lambda function is used within the RobotPublisher class to define the timer_callback. 
This lambda function is responsible for creating and publishing messages periodically.

*/

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  RobotPublisher()
  : Node("robot_publisher"), count_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback =
      [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->pub_->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPublisher>());
  rclcpp::shutdown();
  return 0;
}
