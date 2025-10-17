#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner
{
    void CustomPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
                                  std::string name,
                                  std::shared_ptr<tf2_ros::Buffer> tf,
                                  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
    }

    void CustomPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s of type GlobalPlanner", name_.c_str());
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "Activating plugin %s of type GlobalPlanner", name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s of type GlobalPlanner", name_.c_str());
    }

    nav_msgs::msg::Path CustomPlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        nav_msgs::msg::Path global_path;
        // 进行规划
        return global_path;
    }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)