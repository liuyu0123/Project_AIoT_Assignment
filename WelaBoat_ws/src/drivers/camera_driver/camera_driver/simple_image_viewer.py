#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys


class CameraViewer(Node):
    def __init__(self, topic_name):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        self.topic_name = topic_name
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Subscribed to {topic_name}")

    def image_callback(self, msg):
        try:
            encoding = msg.encoding
            self.get_logger().debug(f"Received image with encoding: {encoding}")

            if encoding in ['bgr8', 'rgb8', 'mono8']:
                # 标准格式：直接显示
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
                if encoding == 'rgb8':
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif encoding == '32FC1':
                # 视差或深度（float32）
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # 过滤无效值（<=0）
                cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)
                mask = cv_image > 0
                if np.any(mask):
                    min_val = cv_image[mask].min()
                    max_val = cv_image[mask].max()
                    if max_val > min_val:
                        cv_image = (cv_image - min_val) / (max_val - min_val) * 255.0
                    else:
                        cv_image = np.zeros_like(cv_image)
                else:
                    cv_image = np.zeros_like(cv_image)
                cv_image = cv_image.astype(np.uint8)
                cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)
            elif encoding == '16UC1':
                # 深度图（毫米级 uint16）
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                # 转为 float 以便归一化
                cv_image = cv_image.astype(np.float32)
                mask = cv_image > 0
                if np.any(mask):
                    min_val = cv_image[mask].min()
                    max_val = cv_image[mask].max()
                    if max_val > min_val:
                        cv_image = (cv_image - min_val) / (max_val - min_val) * 255.0
                    else:
                        cv_image = np.zeros_like(cv_image)
                else:
                    cv_image = np.zeros_like(cv_image)
                cv_image = cv_image.astype(np.uint8)
                cv_image = cv2.applyColorMap(cv_image, cv2.COLORMAP_JET)
            else:
                self.get_logger().error(f"Unsupported encoding: {encoding}")
                return

            cv2.imshow(f"Viewer: {self.topic_name}", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run camera_driver camera_viewer <topic_name>")
        return

    topic_name = sys.argv[1]

    rclpy.init(args=args)
    viewer = CameraViewer(topic_name)
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()