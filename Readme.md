# Beginner Tutorials - ROS 2 Package

## Overview

This repository contains the `beginner_tutorials` ROS 2 package. The main purpose of this package is to introduce the basics of creating and running ROS 2 nodes using the C++ programming language. This package includes a simple publisher node that publishes custom messages to a topic.

The goal is to help beginners understand fundamental concepts in ROS 2, such as creating nodes, publishing messages, and understanding the ROS 2 communication mechanism.

## Features
- A publisher node (`publisher_node.cpp`) that publishes a custom message to a ROS 2 topic.
- The node is implemented in C++ and uses ROS 2 Humble libraries for message passing.

## Assumptions/Dependencies

- **ROS 2 Distribution**: This package has been developed and tested using **ROS 2 Humble**.
- **C++17 Standard**: The code is written using C++17 features, so make sure that your build environment supports C++17.
- **Colcon**: The recommended build tool for ROS 2.
- **clang-tidy** and **cpplint**: Used for code quality and style checking.

Ensure you have ROS 2 Humble properly installed and sourced on your system. For installation instructions, refer to the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

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

To run the publisher node:

1. **Navigate to Your Workspace** (if not already there):
   ```sh
   cd ~/my_beginner_tutorials
   ```

2. **Run the Publisher Node**:
   ```sh
   ros2 run beginner_tutorials publisher_node
   ```

This command will start the publisher node, which will begin publishing custom string messages to the specified topic.

## Code Quality Tools

- **cpplint**: Run `cpplint` to check code style:
  ```sh
  cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/publisher_node.cpp > cpplint_output.txt
  ```

- **clang-tidy**: Run `clang-tidy` to check code quality:
  ```sh
  clang-tidy src/publisher_node.cpp -p /home/sarang/my_beginner_tutorials/build/beginner_tutorials -- -std=c++17
  ```

## License

This package is licensed under the MIT License. See the `LICENSE` file for more information.

