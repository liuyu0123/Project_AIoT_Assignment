#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import yaml
import os
from ament_index_python.packages import get_package_share_directory



class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.state = "IDLE"

        self.get_logger().info("Mission Manager Started")

        # --------------------------
        # 航点列表（map 坐标）
        # --------------------------
        # self.waypoints = [
        #     (2.0, 1.0, 0.0),
        #     (4.0, 3.0, 1.57),
        #     (6.0, 2.0, 3.14)
        # ]
        # --------------------------
        # 读取 YAML 航点
        # --------------------------
        pkg_path = get_package_share_directory('mission_manager')
        mission_file = os.path.join(pkg_path, 'config', 'mission.yaml')

        with open(mission_file, 'r') as f:
            data = yaml.safe_load(f)

        self.waypoints = [
            (wp['x'], wp['y'], wp['yaw'])
            for wp in data['waypoints']
        ]

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from YAML.")
        self.current_index = 0

        # Action Client
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # 等待 action server
        self.get_logger().info("Waiting for Nav2 action server...")
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info("Nav2 action server connected.")

        # 开始任务
        self.send_next_goal()

    # =====================================================
    # 发送下一个航点
    # =====================================================
    def send_next_goal(self):

        if self.current_index >= len(self.waypoints):
            self.state = "FINISHED"
            self.get_logger().info("All waypoints completed. Mission finished.")
            return

        x, y, yaw = self.waypoints[self.current_index]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose(x, y, yaw)

        self.get_logger().info(
            f"Sending waypoint {self.current_index}: "
            f"x={x}, y={y}, yaw={yaw}"
        )

        self.state = "SENDING_GOAL"
        self._send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # =====================================================
    # 创建 PoseStamped
    # =====================================================
    def create_pose(self, x, y, yaw):

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # 简单 yaw 转 quaternion
        import math
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    # =====================================================
    # Goal Response
    # =====================================================
    def goal_response_callback(self, future):

        goal_handle = future.result()

        self.state = "MOVING"
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted.")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    # =====================================================
    # Result Callback
    # =====================================================
    def result_callback(self, future):

        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state = "ARRIVED"
            self.get_logger().info("Waypoint reached successfully.")
        else:
            self.get_logger().warn(f"Goal failed with status: {status}")

        self.current_index += 1
        self.send_next_goal()

    # =====================================================
    # Feedback Callback
    # =====================================================
    def feedback_callback(self, feedback_msg):

        feedback = feedback_msg.feedback
        # 这里可以打印剩余距离等信息（暂时不打印避免刷屏）
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
