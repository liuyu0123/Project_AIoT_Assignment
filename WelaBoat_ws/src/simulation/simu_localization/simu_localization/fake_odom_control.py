#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class FakeOdom(Node):

    def __init__(self):
        super().__init__('fake_odom')

        # ================== 参数 ==================
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 50.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # ================== 状态 ==================
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vyaw = 0.0

        self.last_time = self.get_clock().now()

        # ================== 通信 ==================
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # ================== 定时器 ==================
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info('Fake odom node started')

    # ====================================================
    def cmd_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vyaw = msg.angular.z

    # ====================================================
    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # ========= 积分运动学 =========
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt
        self.yaw += self.vyaw * dt

        # ========= quaternion =========
        qz = math.sin(self.yaw * 0.5)
        qw = math.cos(self.yaw * 0.5)

        # ========= TF =========
        tf = TransformStamped()
        tf.header.stamp = now.to_msg()
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame

        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

        # ========= Odom =========
        odom = Odometry()
        odom.header.stamp = tf.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vyaw

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
