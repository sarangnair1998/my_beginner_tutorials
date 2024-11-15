// Copyright 2024 Sarang

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

/** @file
 *  @brief PublisherNode publishes messages and provides a service to toggle the publishing on or off.
 */

/**
 * @class PublisherNode
 * @brief A ROS2 node that publishes messages periodically, broadcasts a TF transform, and provides a service to toggle publishing on or off.
 */
class PublisherNode : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for PublisherNode.
     * Initializes the publisher, timer, service for toggling publishing, and the TF broadcaster.
     */
    PublisherNode()
        : Node("publisher_node"), message_("Hello, this is Sarang"),
          publishing_enabled_(true) {
        // Declare and get the parameter for publishing frequency
        this->declare_parameter("publish_frequency", 2.0);
        double publish_frequency = this->get_parameter("publish_frequency")
                                       .as_double();

        // Create a publisher to publish to the "topic"
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(
                static_cast<int>(1000.0 / publish_frequency)),
            std::bind(&PublisherNode::publish_message, this));

        // Create a service to toggle publishing on/off
        toggle_publishing_service_ = this->create_service<
        std_srvs::srv::SetBool>(
            "toggle_publishing",
            std::bind(&PublisherNode::handle_toggle_publishing, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Create a TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Timer to periodically broadcast the TF transform
        tf_timer_ = this->create_wall_timer(
            500ms, std::bind(&PublisherNode::broadcast_tf, this));

        RCLCPP_INFO_STREAM(this->get_logger(),
                           "PublisherNode has started. "
                           "Publishing is initially enabled.");
    }

 private:
    /**
     * @brief Publishes a message if publishing is enabled.
     * Checks if publishing is enabled before publishing a message to the topic.
     */
    void publish_message() {
        if (!publishing_enabled_) {
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                                "Publishing is currently disabled.");
            return;
        }

        auto message = std_msgs::msg::String();
        message.data = message_;
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
        publisher_->publish(message);
    }

    /**
     * @brief Handles the toggle publishing service request.
     * @param request The request containing the desired publishing state
     * (true to enable, false to disable).
     * @param response The response indicating whether the request was successful
     * and a message describing the result.
     */
    void handle_toggle_publishing(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        // Update the publishing state based on the service request
        publishing_enabled_ = request->data;

        if (publishing_enabled_) {
            response->message = "Publishing has been enabled.";
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Publishing has been enabled.");
        } else {
            response->message = "Publishing has been disabled.";
            RCLCPP_WARN_STREAM(this->get_logger(),
                               "Publishing has been disabled.");
        }

        response->success = true;
    }

    /**
     * @brief Broadcasts a TF transform with the frame "talk" as child of "world".
     */
    void broadcast_tf() {
        geometry_msgs::msg::TransformStamped transformStamped;
        // Set the header and frame information
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "talk";

        // Set a non-zero translation
        transformStamped.transform.translation.x = 1.0;
        transformStamped.transform.translation.y = 2.0;
        transformStamped.transform.translation.z = 3.0;

        // Set a non-zero rotation (as quaternion)
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.707;  // 90 degrees
        transformStamped.transform.rotation.w = 0.707;

        // Broadcast the transform
        tf_broadcaster_->sendTransform(transformStamped);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
        toggle_publishing_service_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr tf_timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string message_; /**< The message to be published. */
    bool publishing_enabled_;
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
    RCLCPP_DEBUG_STREAM(node->get_logger(),
                        "Initializing PublisherNode...");
    rclcpp::spin(node);
    RCLCPP_FATAL_STREAM(node->get_logger(),
                        "Shutting down PublisherNode due to unexpected error.");
    rclcpp::shutdown();
    return 0;
}
