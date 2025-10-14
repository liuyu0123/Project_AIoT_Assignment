from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration


def main():
    rclpy.init()
    navigator = BasicNavigator()
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    # Set the goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0
    # Send the goal pose to Nav2
    navigator.goToPose(goal_pose)
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        navigator.get_logger().info('Moving towards goal pose...',
                                    f'will be {Duration.from_msg(feedback.estimate_time_remaining).nanoseconds/1e9} seconds after arrive')
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
            print('can not arrive, calculate time over')
            navigator.cancelTask()
    # Check for success
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().warn('Goal was canceled')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Goal failed')
    else:
        navigator.get_logger().error('The result is invalid')
