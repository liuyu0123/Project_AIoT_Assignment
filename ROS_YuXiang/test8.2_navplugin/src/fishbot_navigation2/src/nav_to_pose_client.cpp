#include <memory>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigationAction = nav2_msgs::action::NavigateToPose;
using NavigationActionClient = rclcpp_action::Client<NavigationAction>;
using NavigationActionGoalHandle = rclcpp_action::ClientGoalHandle<NavigationAction>;

class NavToPoseClient : public rclcpp::Node
{
public:
    NavToPoseClient() : Node("nav_to_pose_client")
    {
        action_client_ = rclcpp_action::create_client<NavigationAction>(this, "navigate_to_pose");
    }
    void sendGoal()
    {
        while (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the action server to come up");
        }

        auto goal_msg = NavigationAction::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = 2.0;
        goal_msg.pose.pose.position.y = 2.0;
        goal_msg.pose.pose.orientation.w = 1.0;

        auto send_goal_options = rclcpp_action::Client<NavigationAction>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this](std::shared_ptr<NavigationActionGoalHandle> goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.feedback_callback = [this](std::shared_ptr<NavigationActionGoalHandle> goal_handle,
                                                     const std::shared_ptr<const NavigationAction::Feedback> feedback)
        {
            (void)goal_handle;
            RCLCPP_INFO(this->get_logger(), "Feedback remain distance: %f", feedback->distance_remaining);
        };

        send_goal_options.result_callback = [this](const NavigationActionGoalHandle::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED){
                RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavToPoseClient>();
    node->sendGoal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}