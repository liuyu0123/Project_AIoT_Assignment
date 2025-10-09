import math
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_tf()

    def publish_static_tf(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'camera_link'
        static_transform.transform.translation.x = 0.5
        static_transform.transform.translation.y = 0.3
        static_transform.transform.translation.z = 0.6
        # 旋转
        rotation_quat = quaternion_from_euler(math.radians(180), 0, 0)
        static_transform.transform.rotation.x = rotation_quat[0]
        static_transform.transform.rotation.y = rotation_quat[1]
        static_transform.transform.rotation.z = rotation_quat[2]
        static_transform.transform.rotation.w = rotation_quat[3]
        # 发布
        self.static_broadcaster_.sendTransform(static_transform)
        self.get_logger().info(f'Publish static tf {static_transform}')


def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()