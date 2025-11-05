#ifndef MINIMAL_TOPIC_TALKER_NODE_HPP_
#define MINIMAL_TOPIC_TALKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class TalkerNode : public rclcpp::Node
{
public:
    explicit TalkerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});
    ~TalkerNode() = default;

private:
    void timer_callback();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string message_; // 可参数化
    std::chrono::milliseconds period_;
};

#endif // MINIMAL_TOPIC_TALKER_NODE_HPP_