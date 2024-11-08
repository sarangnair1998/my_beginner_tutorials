// Copyright 2024 Sarang

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

/** @file
 *  @brief PublisherNode publishes messages and provides a service to toggle the publishing on or off.
 */

/**
 * @class PublisherNode
 * @brief A ROS2 node that publishes messages periodically and provides a service to toggle publishing on or off.
 */
class PublisherNode : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for PublisherNode.
     * Initializes the publisher, timer, and service for toggling publishing.
     */
    PublisherNode() : Node("publisher_node"), message_("Hello, this is Sarang"), publishing_enabled_(true) {
        // Declare and get the parameter for publishing frequency
        this->declare_parameter("publish_frequency", 2.0);
        double publish_frequency = this->get_parameter("publish_frequency").as_double();

        // Create a publisher to publish to the "topic"
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_frequency)),
            std::bind(&PublisherNode::publish_message, this)
        );

        // Create a service to toggle publishing on/off
        toggle_publishing_service_ = this->create_service<std_srvs::srv::SetBool>(
            "toggle_publishing", 
            std::bind(&PublisherNode::handle_toggle_publishing, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO_STREAM(this->get_logger(), "PublisherNode has started. Publishing is initially enabled.");
    }

 private:
    /**
     * @brief Publishes a message if publishing is enabled.
     * Checks if publishing is enabled before publishing a message to the topic.
     */
    void publish_message() {
        if (!publishing_enabled_) {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing is currently disabled.");
            return;
        }

        auto message = std_msgs::msg::String();
        message.data = message_;
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
        publisher_->publish(message);
    }

    /**
     * @brief Handles the toggle publishing service request.
     * @param request The request containing the desired publishing state (true to enable, false to disable).
     * @param response The response indicating whether the request was successful and a message describing the result.
     */
    void handle_toggle_publishing(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        // Update the publishing state based on the service request
        publishing_enabled_ = request->data;

        if (publishing_enabled_) {
            response->message = "Publishing has been enabled.";
            RCLCPP_WARN_STREAM(this->get_logger(), "Publishing has been enabled.");
        } else {
            response->message = "Publishing has been disabled.";
            RCLCPP_WARN_STREAM(this->get_logger(), "Publishing has been disabled.");
        }

        response->success = true;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; /**< Publisher to publish messages to the topic. */
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_publishing_service_; /**< Service to toggle publishing on or off. */
    rclcpp::TimerBase::SharedPtr timer_; /**< Timer to control the publishing interval. */
    std::string message_; /**< The message to be published. */
    bool publishing_enabled_; /**< State to keep track if publishing is enabled or disabled. */
};

/**
 * @brief Main function to initialize and spin the PublisherNode.
 * @param argc Argument count.
 * @param argv Argument values.
 * @return int Exit status code.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    RCLCPP_DEBUG_STREAM(node->get_logger(), "Initializing PublisherNode...");
    rclcpp::spin(node);
    RCLCPP_FATAL_STREAM(node->get_logger(), "Shutting down PublisherNode due to unexpected error.");
    rclcpp::shutdown();
    return 0;
}
