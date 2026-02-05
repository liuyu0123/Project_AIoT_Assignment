#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time

class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.5      # m/s
        self.w = 0.2      # rad/s
        self.last_time = time.time()

        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        self.pub.publish(msg)

        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(FakeOdom())
    rclpy.shutdown()
