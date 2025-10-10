#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "turtle1/cmd_vel", 10);
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&TurtleController::on_pose_received2_, this, std::placeholders::_1));
    }

private:
    void on_pose_received_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto message = geometry_msgs::msg::Twist();
        // 1. 当前位置
        double current_x = pose->x;
        double current_y = pose->y;
        RCLCPP_INFO(this->get_logger(), "Current Location at x: %f, y: %f", current_x, current_y);

        // 2. 计算目标距离
        double distance = sqrt(pow(current_x - target_x_, 2) + pow(current_y - target_y_, 2));
        double angle = atan2(current_y - target_y_, current_x - target_x_) - pose->theta;

        // 3. 控制策略
        if (distance > 0.1)
        {
            if (fabs(angle) > 0.05)
            {
                message.angular.z = fabs(angle);
            }
            else
            {
                message.linear.x = k_ * distance;
            }
        }

        // 4. 约束最大值，发送消息
        if (message.linear.x > max_speed_)
        {
            message.linear.x = max_speed_;
        }
        velocity_publisher_->publish(message);
    }

    void on_pose_received2_(const turtlesim::msg::Pose::SharedPtr pose)
    {
        auto cmd = geometry_msgs::msg::Twist();

        const double dx = target_x_ - pose->x;
        const double dy = target_y_ - pose->y;
        const double distance = std::hypot(dx, dy);
        const double target_theta = std::atan2(dy, dx);
        double angle_err = target_theta - pose->theta;

        // 角度正规化到 [-pi, pi]
        angle_err = std::atan2(std::sin(angle_err), std::cos(angle_err));

        RCLCPP_INFO(this->get_logger(),
                    "dist=%.2f  angle_err=%.2f rad", distance, angle_err);

        if (distance < 0.05)
        { // 到达目标，停
            velocity_publisher_->publish(cmd);
            return;
        }

        // 先转，再前进；可同时进行
        cmd.angular.z = std::clamp(angle_err * 3.0, -max_angular_, max_angular_);
        cmd.linear.x = std::clamp(distance * k_, 0.0, max_speed_);

        velocity_publisher_->publish(cmd);
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    double target_x_{1.0};
    double target_y_{1.0};
    double k_{1.0};
    double max_speed_{3.0};
    double max_angular_{2.0}; // rad/s
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}