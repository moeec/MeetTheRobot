/*
ROS 2 Publisher Overview (RobotPublisher)

This node publishes a std_msgs::msg::String message on the topic "topic"
at a fixed interval of 500 ms (2 Hz) using a wall-clock timer.

Step-by-step behavior:
1) Create Publisher
   - Type: std_msgs::msg::String
   - Topic: "topic"
   - QoS: KeepLast(10) with default Reliable / Volatile settings

2) Define Timer Callback (lambda)
   - Signature: [this]() -> void
   - The [this] capture lets the lambda access node members (pub_, count_) and
     methods (get_logger()).

3) Create & Publish Message in the Callback
   - Construct a std_msgs::msg::String message.
   - Set message.data = "Hello, world! " + std::to_string(count_++),
     appending an incrementing counter each time the callback runs.
   - Log the outgoing message with RCLCPP_INFO.
   - Publish the message via pub_.

Timing semantics:
- Uses create_wall_timer(500ms, ...) which is based on wall-clock time,
  not ROS simulated time. If simulated time is needed, prefer a timer bound
  to the node clock and enable use_sim_time.

Node details:
- Class: RobotPublisher
- Base node name: "robot_publisher"

In summary, this node periodically publishes a human-readable string with an
incrementing counter, which is useful for debugging, testing connectivity,
and providing regular heartbeat-style updates to other nodes.
*/


#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class RobotPublisher : public rclcpp::Node
{
public:
  RobotPublisher()
  : rclcpp::Node("robot_publisher"), count_(0)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    auto timer_callback = [this]() {
      std_msgs::msg::String message;
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      pub_->publish(message);
    };

    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  size_t count_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPublisher>());
  rclcpp::shutdown();
  return 0;
}
