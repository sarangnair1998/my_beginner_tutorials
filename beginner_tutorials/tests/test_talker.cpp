// Copyright 2024 Sarang

/// @file test_talker.cpp
/// @brief Integration test for the Talker node using Catch2.

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <thread>

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
class MyTestsFixture {
 public:
  MyTestsFixture() {
    rclcpp::init(0, nullptr);
    // Create the node that performs the test
    testerNode = rclcpp::Node::make_shared("IntegrationTestNode");

    // Declare a parameter for the duration of the test
    testerNode->declare_parameter<double>("test_duration", 5.0);

    // Get the test duration value
    TEST_DURATION = testerNode->get_parameter("test_duration").as_double();
    RCLCPP_INFO_STREAM(testerNode->get_logger(),
     "Got test_duration = " << TEST_DURATION);
  }

  ~MyTestsFixture() {
    rclcpp::shutdown();
  }

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

////////////////////////////////////////////////
// Test Case 1: Test Topic Publishing
////////////////////////////////////////////////
TEST_CASE_METHOD(MyTestsFixture,
"PublisherNode publishes messages", "[talker]") {
  // Create a node to listen to the "chatter" topic
  auto subscriber_node = rclcpp::Node::make_shared("test_subscriber_node");
  bool message_received = false;

  auto subscription = subscriber_node->
  create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [&message_received](std_msgs::msg::String::ConstSharedPtr msg) {
        // Assert that the message data is as expected
        REQUIRE(msg->data == "Hello, this is Sarang");
        message_received = true;
      });

  // Create a node to publish messages
  auto publisher_node =
  rclcpp::Node::make_shared("publisher_node");
  auto publisher = publisher_node->create_publisher
  <std_msgs::msg::String>("chatter", 10);

  // Create an executor to spin the nodes
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(subscriber_node);
  executor.add_node(publisher_node);

  // Wait for a moment to ensure both nodes are ready
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // Publish a message to simulate the Talker behavior
  auto message = std_msgs::msg::String();
  message.data = "Hello, this is Sarang";
  publisher->publish(message);

  // Spin for a few seconds to check if messages are published
  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() -
  start_time < std::chrono::seconds(5)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (message_received) {
      break;
    }
  }

  // Ensure that the message was received
  CHECK(message_received);
}

////////////////////////////////////////////////
// Test Case 2: Test TF Broadcast
////////////////////////////////////////////////
TEST_CASE_METHOD(MyTestsFixture,
"PublisherNode broadcasts TF transform", "[talker_tf]") {
  // Create a node with a TF2 buffer and transform listener
  auto tf_node = rclcpp::Node::make_shared("tf_listener_node");
  tf2_ros::Buffer tf_buffer(tf_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Create an executor to spin the node
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(tf_node);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Spin for a few seconds to allow transforms to be available
  auto start_time = std::chrono::steady_clock::now();
  bool transform_found = false;
  while (std::chrono::steady_clock::now() -
  start_time < std::chrono::seconds(10)) {
    executor.spin_some();
    try {
      // Try to get the transform between /world and /talk
      geometry_msgs::msg::TransformStamped transform_stamped;
      transform_stamped = tf_buffer.lookupTransform
      ("world", "talk", tf2::TimePointZero);
      // Verify translation and rotation values
      REQUIRE(transform_stamped.transform.translation.x ==
      Approx(1.0).margin(0.01));
      REQUIRE(transform_stamped.transform.translation.y ==
      Approx(2.0).margin(0.01));
      REQUIRE(transform_stamped.transform.translation.z ==
      Approx(3.0).margin(0.01));
      REQUIRE(transform_stamped.transform.rotation.z ==
      Approx(0.707).margin(0.001));
      REQUIRE(transform_stamped.transform.rotation.w ==
      Approx(0.707).margin(0.001));

      transform_found = true;
      break;
    } catch (const tf2::TransformException &ex) {
      // Transform not yet available
      RCLCPP_WARN(tf_node->get_logger(),
      "Transform not yet available: %s", ex.what());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Ensure that the transform was found
  CHECK(transform_found);
}
