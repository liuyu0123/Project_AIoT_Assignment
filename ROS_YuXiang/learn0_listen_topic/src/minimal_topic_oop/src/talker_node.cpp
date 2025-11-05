#include "minimal_topic_oop/talker_node.hpp"

using namespace std::chrono_literals;

TalkerNode::TalkerNode(const rclcpp::NodeOptions &options)
    : Node("talker", options),
      message_(declare_parameter("message", "Hello, ROS 2 topic!")),
      period_(std::chrono::milliseconds(declare_parameter("period_ms", 1000)))
{
    pub_ = create_publisher<std_msgs::msg::String>("hello", 10);
    timer_ = create_wall_timer(
        period_, [this]()
        { timer_callback(); });
}

void TalkerNode::timer_callback()
{
    auto msg = std_msgs::msg::String();
    msg.data = message_;
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published: '%s'", msg.data.c_str());
}