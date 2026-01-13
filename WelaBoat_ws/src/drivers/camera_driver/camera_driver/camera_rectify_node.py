# camera_driver/camera_driver/rectify_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from .utils.camera_configs_auto import CamConf  # 导入相机标定参数


class StereoRectifyNode(Node):
    def __init__(self):
        super().__init__('stereo_rectify_node')

        # 加载校正映射表（只加载一次）
        self.left_map1 = CamConf['left_map1']
        self.left_map2 = CamConf['left_map2']
        self.right_map1 = CamConf['right_map1']
        self.right_map2 = CamConf['right_map2']

        self.bridge = CvBridge()

        # 订阅原始图像
        self.left_sub = self.create_subscription(
            Image, '/camera/left/image_raw', self.left_callback, 10)
        self.right_sub = self.create_subscription(
            Image, '/camera/right/image_raw', self.right_callback, 10)

        # 发布校正后图像
        self.left_pub = self.create_publisher(Image, '/camera/left/image_rect', 10)
        self.right_pub = self.create_publisher(Image, '/camera/right/image_rect', 10)

        # 缓存最新帧（用于同步，简单版）
        self.latest_left = None
        self.latest_right = None
        self.left_stamp = None
        self.right_stamp = None

        self.get_logger().info("Stereo rectify node started.")

    def left_callback(self, msg):
        self.latest_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left_stamp = msg.header.stamp
        self.try_publish()

    def right_callback(self, msg):
        self.latest_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right_stamp = msg.header.stamp
        self.try_publish()

    def try_publish(self):
        # 简单时间戳匹配（容忍 1ms 误差）
        if self.latest_left is not None and self.latest_right is not None:
            if abs(self.left_stamp.nanosec - self.right_stamp.nanosec) < 1_000_000:  # 1ms
                # 校正
                left_rect = cv2.remap(self.latest_left, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
                right_rect = cv2.remap(self.latest_right, self.right_map1, self.right_map2, cv2.INTER_LINEAR)

                # 转回 ROS Image
                left_msg = self.bridge.cv2_to_imgmsg(left_rect, encoding='bgr8')
                right_msg = self.bridge.cv2_to_imgmsg(right_rect, encoding='bgr8')
                left_msg.header.stamp = self.left_stamp
                right_msg.header.stamp = self.right_stamp
                left_msg.header.frame_id = "camera_left"
                right_msg.header.frame_id = "camera_right"

                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)

                # 清空缓存（避免重复发布）
                self.latest_left = None
                self.latest_right = None


def main(args=None):
    rclpy.init(args=args)
    node = StereoRectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()