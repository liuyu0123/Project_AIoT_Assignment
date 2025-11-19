#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
    OdomTopic2TF(std::string name) : Node(name) {
        odom_subscribe_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), 
            std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1)
        );

        tf_boardcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_boardcaster_;

    void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;
        transform.header = msg->header;
        transform.child_frame_id = msg->child_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation.x = msg->pose.pose.orientation.x;
        transform.transform.rotation.y = msg->pose.pose.orientation.y;
        transform.transform.rotation.z = msg->pose.pose.orientation.z;
        transform.transform.rotation.w = msg->pose.pose.orientation.w;

        tf_boardcaster_->sendTransform(transform);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomTopic2TF>("odom2tf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}