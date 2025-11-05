#ifndef MINIMAL_TOPIC_LISTENER_NODE_HPP_
#define MINIMAL_TOPIC_LISTENER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ListenerNode : public rclcpp::Node
{
public:
    explicit ListenerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});
    ~ListenerNode() = default;

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

#endif // MINIMAL_TOPIC_LISTENER_NODE_HPP_