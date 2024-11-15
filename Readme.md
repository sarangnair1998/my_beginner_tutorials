# Beginner Tutorials - ROS 2 Package

## Overview

This repository contains the `beginner_tutorials` ROS 2 package. The main purpose of this package is to introduce the basics of creating and running ROS 2 nodes using the C++ programming language. This package includes a publisher node that publishes custom messages to a topic, a subscriber node that receives those messages, a service to toggle the publishing on or off, integration tests, and the ability to record and replay ROS bag files.

## Features

- **Publisher Node (`publisher_node.cpp`)**:
  - Publishes a custom message to a ROS 2 topic.
  - Provides a service `/toggle_publishing` to enable or disable message publishing.
  - Uses all five ROS logging levels (DEBUG, INFO, WARN, ERROR, FATAL).
  - Broadcasts a TF frame called `/talk` with parent `/world`, with non-zero translation and rotation.

- **Subscriber Node (`subscriber_node.cpp`)**:
  - Subscribes to the published topic and logs received messages.
  - Detects inactivity and logs a warning if no messages are received for a certain period.

- **Launch File (`publisher_subscriber_launch.py`)**:
  - Launches both publisher and subscriber nodes.
  - Allows users to set the `publish_frequency` parameter for the publisher node via command-line arguments.

- **Bag Recording Launch File (`ros_bag_launch_file.launch.py`)**:
  - Launches the publisher and subscriber nodes with an optional argument to record all topics to a ROS bag file.

- **Integration Tests (`test_talker.cpp`)**:
  - Integration tests using Catch2 to verify topic publishing and TF frame broadcasting.

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

4. **Run Both Nodes with Bag Recording Enabled**:
   To run the publisher and subscriber nodes and record all topics to a bag file:
   ```sh
   ros2 launch beginner_tutorials publisher_subscriber_launch.py record:=true
   ```

5. **Inspecting TF Frames**:
   To inspect the TF frames being broadcasted by the publisher node:
   ```sh
   ros2 run tf2_ros tf2_echo world talk
   ```
   You can also generate a PDF representation of the TF tree:
   ```sh
   ros2 run tf2_tools view_frames
   ```
   The output will be saved as `frames.pdf`.

6. **Replay Bag File**:
   To replay the recorded bag file and verify the messages:
   - Run the subscriber node:
     ```sh
     ros2 run beginner_tutorials subscriber_node
     ```
   - Replay the bag file:
     ```sh
     ros2 bag play results/
     ```

7. **Inspecting Bag File**:
   To verify the contents of the recorded bag file:
   ```sh
   ros2 bag info results/
   ```
   This command will display information about the topics, message types, and duration of the bag file.

## Running Tests

To run the integration tests, first ensure the `publisher_node` is running. The tests depend on the messages and TF frames being published.

1. **Run the Publisher Node**:
   ```sh
   source install/setup.bash
   ros2 run beginner_tutorials publisher_node
   ```

2. **Run the Integration Test**:
   In another terminal, run the test executable directly:
   ```sh
   source install/setup.bash
   ./build/beginner_tutorials/test_talker
   ```

**Note**: Running `colcon test` directly will not work as it does not start the necessary publisher node before executing the tests.

## License

This package is licensed under the MIT License. See the `LICENSE` file for more information.
