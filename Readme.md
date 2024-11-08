# Beginner Tutorials - ROS 2 Package

## Overview

This repository contains the `beginner_tutorials` ROS 2 package. The main purpose of this package is to introduce the basics of creating and running ROS 2 nodes using the C++ programming language. This package includes a publisher node that publishes custom messages to a topic, a subscriber node that receives those messages, and a service to toggle the publishing on or off.

## Features

- **Publisher Node (`publisher_node.cpp`)**:
  - Publishes a custom message to a ROS 2 topic.
  - Provides a service `/toggle_publishing` to enable or disable message publishing.
  - Uses all five ROS logging levels (DEBUG, INFO, WARN, ERROR, FATAL).

- **Subscriber Node (`subscriber_node.cpp`)**:
  - Subscribes to the published topic and logs received messages.
  - Detects inactivity and logs a warning if no messages are received for a certain period.

- **Launch File (`publisher_subscriber_launch.py`)**:
  - Launches both publisher and subscriber nodes.
  - Allows users to set the `publish_frequency` parameter for the publisher node via command-line arguments.

## Assumptions/Dependencies

- **ROS 2 Distribution**: This package has been developed and tested using **ROS 2 Humble**.
- **C++17 Standard**: The code is written using C++17 features, so ensure that your build environment supports C++17.
- **Colcon**: The recommended build tool for ROS 2.
- **clang-tidy** and **cpplint**: Used for code quality and style checking.

## Build Steps

To build this package, follow these steps:

1. **Source ROS 2 Setup**:
   ```sh
   source /opt/ros/humble/setup.bash
   ```

2. **Navigate to Your ROS 2 Workspace**:
   ```sh
   cd ~/my_beginner_tutorials
   ```

3. **Build the Package**:
   ```sh
   colcon build --packages-select beginner_tutorials
   ```

4. **Source the Workspace**:
   After building, source the workspace to add the package to your environment:
   ```sh
   source install/setup.bash
   ```

## Run Steps


1. **Navigate to Your Workspace** (if not already there):
   ```sh
   cd ~/my_beginner_tutorials
   ```

2. **Run Both Nodes Using the Launch File**:
   ```sh
   ros2 launch beginner_tutorials publisher_subscriber_launch.py

   ```
   You can also modify the publish_frequency parameter by passing it as a command-line argument:
   ```sh
   ros2 launch beginner_tutorials publisher_subscriber_launch.py publish_frequency:=5.0

   ```

3. **Using the Service to Toggle Publishing**:

   The publisher node provides a service to enable or disable message publishing. Use the following commands:

   Disable Publishing:

   ```sh
   ros2 service call /toggle_publishing std_srvs/srv/SetBool "{data: false}"
   ```

   Enable Publishing:

   ```sh
   ros2 service call /toggle_publishing std_srvs/srv/SetBool "{data: true}"
   ```



## License

This package is licensed under the MIT License. See the `LICENSE` file for more information.



