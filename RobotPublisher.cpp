#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/*

This code sets up a ROS 2 publisher that periodically sends a message to a topic. Here's a step-by-step summary of what happens:

Create Publisher: A publisher is created for a topic named "topic" with a message type of std_msgs::msg::String and a queue size of 10.
Define Timer Callback: A lambda function is defined as a timer callback.
Create and Publish Message: Within the lambda function:
A new std_msgs::msg::String message is created.
The data field of the message is set to "Hello, world! " followed by the current value of count_, which is then incremented.
An informational log message is printed with the message content.
The message is published to the topic.
This setup allows the node to regularly publish a string message with an incrementing counter to the specified topic, which can be 
useful for debugging or for regular updates to other nodes in the system.

The lambda function is used within the RobotPublisher class to define the timer_callback. 
This lambda function is responsible for creating and publishing messages periodically.

[this]: This capture clause allows the lambda function to capture the this pointer, 
enabling it to access the member variables (pub_ and count_) and member functions 
(get_logger()) of the RobotPublisher class.
() -> void: This specifies that the lambda function takes no parameters and returns void.
The function body creates a message, sets its data, logs the message, and publishes it.

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
