// Copyright 2024 Sarang

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

using namespace std::chrono_literals;

/** @file
 *  @brief SubscriberNode subscribes to the "topic" and logs the received messages, including inactivity status.
 */

/**
 * @class SubscriberNode
 * @brief A ROS2 node that subscribes to the "topic" and logs incoming messages, and detects inactivity.
 */
class SubscriberNode : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for SubscriberNode.
     * Initializes the subscriber to subscribe to the "topic" and sets up a timer
     * to monitor activity.
     */
    SubscriberNode() : Node("subscriber_node"), last_message_time_(now()) {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&SubscriberNode::topic_callback, this,
                      std::placeholders::_1));

        // Create a timer to check for inactivity
        inactivity_timer_ = this->create_wall_timer(
            1000ms, std::bind(&SubscriberNode::check_inactivity, this));

        RCLCPP_INFO_STREAM(this->get_logger(), "SubscriberNode has started.");
    }

 private:
    /**
     * @brief Callback function to process incoming messages.
     * @param msg The incoming message of type std_msgs::msg::String.
     */
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO_STREAM(this->get_logger(),
        "Received message: " << msg->data);
        last_message_time_ = now();  // Update last received message time
    }

    /**
     * @brief Checks if no messages have been received for a period of time
     * and logs a warning if idle.
     */
    void check_inactivity() {
        auto current_time = now();
        auto time_since_last_msg =
            std::chrono::duration_cast<std::chrono::seconds>(
                current_time - last_message_time_)
                .count();

        if (time_since_last_msg > 2) {
            // If more than 2 seconds have passed since the last message
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "No messages received in the last "
                               << time_since_last_msg << " seconds.");
        }
    }

    /**
     * @brief Helper function to get the current time.
     * @return std::chrono::steady_clock::time_point The current time.
     */
    std::chrono::steady_clock::time_point now() {
        return std::chrono::steady_clock::now();
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr inactivity_timer_;
    std::chrono::steady_clock::time_point last_message_time_;
};

/**
 * @brief Main function to initialize and spin the SubscriberNode.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return int Exit status code.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
