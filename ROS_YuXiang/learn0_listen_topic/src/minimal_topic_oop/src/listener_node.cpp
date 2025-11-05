#include "minimal_topic_oop/listener_node.hpp"

ListenerNode::ListenerNode(const rclcpp::NodeOptions & options)
: Node("listener", options)
{
  sub_ = create_subscription<std_msgs::msg::String>(
    "hello", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      topic_callback(msg);
    });
}

void ListenerNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "I heard: '%s'", msg->data.c_str());
}