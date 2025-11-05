#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker");
  auto pub  = node->create_publisher<std_msgs::msg::String>("hello", 10);
  rclcpp::Rate loop(1);          // 1 Hz

  while (rclcpp::ok()) {
    auto msg = std_msgs::msg::String();
    msg.data = "Hello, ROS 2 topic!";
    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Published: %s", msg.data.c_str());
    rclcpp::spin_some(node);
    loop.sleep();
  }
  rclcpp::shutdown();
  return 0;
}