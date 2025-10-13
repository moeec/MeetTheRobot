# MeetTheRobot

# Robot Publisher/Subscriber (ROS 2)

Simple ROS 2 example with a **String publisher** and a **String subscriber** that communicate over the topic `/topic`. The publisher emits a message every **500 ms**; the subscriber logs whatever it receives.

## Overview

* **Publisher (`RobotPublisher`)**

  * Node name: `robot_publisher`
  * Topic: `/topic`
  * Type: `std_msgs/msg/String`
  * Rate: **2 Hz** (every `500ms`)
  * Payload: `"Hello, world! <N>"`, with an incrementing counter
  * Logs each publish with `RCLCPP_INFO`

* **Subscriber (`RobotSubscriber`)**

  * Node name: `robot_subscriber`
  * Subscribes to: `/topic`
  * Type: `std_msgs/msg/String`
  * Callback arg: `std_msgs::msg::String::UniquePtr`
  * Logs each received message with `RCLCPP_INFO`

Both nodes use the default QoS (depth = 10, reliable, volatile) and run on a single-threaded executor via `rclcpp::spin`.

## Requirements

* ROS 2 (any recent distro that supports `rclcpp` and `std_msgs`)
* C++14 or newer
* Package dependencies:

  * `rclcpp`
  * `std_msgs`

In `package.xml`:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

In `CMakeLists.txt` (snippet):

```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_publisher src/robot_publisher.cpp)
ament_target_dependencies(robot_publisher rclcpp std_msgs)

add_executable(robot_subscriber src/robot_subscriber.cpp)
ament_target_dependencies(robot_subscriber rclcpp std_msgs)

install(TARGETS
  robot_publisher
  robot_subscriber
  DESTINATION lib/${PROJECT_NAME})
```

## Build

From the workspace root:

```bash
colcon build --packages-select <your_package_name>
source install/setup.bash   # or setup.zsh
```

## Run

Open two terminals and source your workspace in each.

**Terminal A — Publisher**

```bash
ros2 run <your_package_name> robot_publisher
```

**Terminal B — Subscriber**

```bash
ros2 run <your_package_name> robot_subscriber
```

You should see logs like:

* Publisher:

  ```
  [INFO] [robot_publisher]: Publishing: 'Hello, world! 0'
  [INFO] [robot_publisher]: Publishing: 'Hello, world! 1'
  ...
  ```

* Subscriber:

  ```
  [INFO] [robot_subscriber]: I heard: 'Hello, world! 0'
  [INFO] [robot_subscriber]: I heard: 'Hello, world! 1'
  ...
  ```

(Optional) You can also observe messages with:

```bash
ros2 topic echo /topic
```

## Files

* `src/robot_publisher.cpp`

  * Creates a `std_msgs::msg::String` publisher on `/topic`.
  * Uses a wall timer (`create_wall_timer(500ms, ...)`) to publish at 2 Hz.
  * Increments and appends a counter to each message.

* `src/robot_subscriber.cpp`

  * Creates a subscription to `/topic`.
  * Uses a lambda taking `std_msgs::msg::String::UniquePtr` for zero-copy friendliness.
  * Logs each received message.

## Notes & Tips

* **QoS:** Defaults are compatible between these nodes. For lossy networks or simulated playback, you may need to adjust QoS (e.g., `best_effort()` or `transient_local()`).
* **Timer clock:** `create_wall_timer` uses wall-clock time. If you need simulated time, ensure `use_sim_time` is set appropriately and consider time-aware timers.
* **Naming consistency:** Ensure class names and `std::make_shared<...>` match (e.g., `RobotPublisher`, `RobotSubscriber`).

## Troubleshooting

* No messages in the subscriber?

  * Verify both nodes are running and sourced in the same workspace.
  * Check topic names with `ros2 topic list` (ensure both use `/topic`).
  * Inspect QoS compatibility with `ros2 doctor` or try echoing the topic.
