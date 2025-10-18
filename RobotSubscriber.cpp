/*
ROS 2 Subscriber Overview (RobotSubscriber)

This node subscribes to std_msgs::msg::String messages on the topic "topic"
and logs each message it receives.

Step-by-step behavior:
1) Create Subscriber
   - Type: std_msgs::msg::String
   - Topic: "topic"
   - QoS: KeepLast(10) with default Reliable / Volatile settings
   - The topic name and type must match the publisher.

2) Define Subscription Callback (lambda)
   - Signature: [this](std_msgs::msg::String::UniquePtr msg) -> void
   - The [this] capture allows the lambda to use the node's logger via get_logger().
   - The parameter is a UniquePtr to the message:
       * Ownership is transferred to the callback (zero-copy friendly for intra-process paths).
       * Do not store the pointer beyond the callback; copy data if needed.
   - The callback logs the received string:
       RCLCPP_INFO(logger, "I heard: '%s'", msg->data.c_str());

3) Executor / runtime (in main)
   - rclcpp::init(...) initializes ROS 2.
   - rclcpp::spin(std::make_shared<RobotSubscriber>()) enters the executor loop
     so incoming messages trigger the callback.
   - rclcpp::shutdown() cleans up on exit.

Notes:
- If the publisher isn't running, the subscriber simply waits; no errors are produced.
- Default QoS here (reliable, volatile, depth 10) is compatible with the paired publisher
  that uses the same defaults.
- Use `ros2 topic list` and `ros2 topic echo /topic` to verify connectivity during testing.

Class / Node details:
- Class: RobotSubscriber
- Base node name: "robot_subscriber"
- Member: sub_ keeps the subscription alive for the lifetime of the node.
*/

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

