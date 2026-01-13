#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber


class StereoDisparityNode(Node):
    def __init__(self):
        super().__init__('stereo_disparity_node')
        self.bridge = CvBridge()

        # === 声明 SGBM 参数 ===
        self.declare_parameter('sgbm.min_disparity', 0)
        self.declare_parameter('sgbm.num_disparities', 64)
        self.declare_parameter('sgbm.block_size', 11)
        self.declare_parameter('sgbm.P1', 8 * 3 * 11**2)
        self.declare_parameter('sgbm.P2', 32 * 3 * 11**2)
        self.declare_parameter('sgbm.disp12_max_diff', 1)
        self.declare_parameter('sgbm.pre_filter_cap', 63)
        self.declare_parameter('sgbm.uniqueness_ratio', 15)
        self.declare_parameter('sgbm.speckle_window_size', 100)
        self.declare_parameter('sgbm.speckle_range', 32)
        self.declare_parameter('sgbm.mode', cv2.STEREO_SGBM_MODE_SGBM_3WAY)

        # === 声明 Q 矩阵参数（从 YAML 中读取）===
        self.declare_parameter('Q_matrix', [1.0, 0.0, 0.0, -320.0,
                                            0.0, 1.0, 0.0, -240.0,
                                            0.0, 0.0, 0.0, 500.0,
                                            0.0, 0.0, 1.0/100.0, 0.0])
        q_list = self.get_parameter('Q_matrix').value
        self.Q = np.array(q_list, dtype=np.float32).reshape(4, 4)

        # === 初始化 SGBM ===
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=self.get_parameter('sgbm.min_disparity').value,
            numDisparities=self.get_parameter('sgbm.num_disparities').value,
            blockSize=self.get_parameter('sgbm.block_size').value,
            P1=self.get_parameter('sgbm.P1').value,
            P2=self.get_parameter('sgbm.P2').value,
            disp12MaxDiff=self.get_parameter('sgbm.disp12_max_diff').value,
            preFilterCap=self.get_parameter('sgbm.pre_filter_cap').value,
            uniquenessRatio=self.get_parameter('sgbm.uniqueness_ratio').value,
            speckleWindowSize=self.get_parameter('sgbm.speckle_window_size').value,
            speckleRange=self.get_parameter('sgbm.speckle_range').value,
            mode=self.get_parameter('sgbm.mode').value
        )

        # === 创建 Publishers ===
        self.disparity_pub = self.create_publisher(Image, '/stereo/disparity', 10)
        self.depth_pub = self.create_publisher(Image, '/stereo/depth', 10)
        self.disparity_color_pub = self.create_publisher(Image, '/stereo/disparity_color', 10)

        # === 创建 Subscribers 并同步 ===
        left_sub = Subscriber(self, Image, '/camera/left/image_rect')
        right_sub = Subscriber(self, Image, '/camera/right/image_rect')

        self.ts = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.stereo_callback)

        self.get_logger().info("Stereo disparity node started. Subscribed to rectified images.")

    def stereo_callback(self, left_msg, right_msg):
        try:
            # 转换为 OpenCV 图像
            left_cv = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='mono8')
            right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='mono8')

            # 计算视差
            disparity = self.sgbm.compute(left_cv, right_cv).astype(np.float32) / 16.0  # SGBM 输出需除以16

            # 发布原始视差图（32FC1）
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity, encoding="32FC1")
            disparity_msg.header = left_msg.header
            self.disparity_pub.publish(disparity_msg)

            # 计算深度图 (Z = f*B / d)
            # 注意：这里使用 Q 矩阵第3行第3列作为基线*f 的倒数（通常 Q[3,2] = 1/(f*B)）
            # 更稳健的方式是用 cv2.reprojectImageTo3D，但此处简化
            depth_method = 1
            if depth_method == 1:
                with np.errstate(divide='ignore'):
                    depth = np.where(disparity > 0, 1.0 / disparity, 0.0)
                depth = (depth * 1000).astype(np.uint16)  # 转为毫米级 16UC1（常见格式）
            elif depth_method == 2:
                # @TODO 这里不正确，还没调试未完成
                points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
                depth = points_3d[:, :, 2]  # Z 通道
                depth[depth <= 0] = 0.0    # 无效深度设为 0
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="16UC1")
            depth_msg.header = left_msg.header
            self.depth_pub.publish(depth_msg)

            # 伪彩色视差图（用于可视化）
            disparity_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
            disparity_color = cv2.applyColorMap(disparity_norm.astype(np.uint8), cv2.COLORMAP_JET)
            disparity_color_msg = self.bridge.cv2_to_imgmsg(disparity_color, encoding="bgr8")
            disparity_color_msg.header = left_msg.header
            self.disparity_color_pub.publish(disparity_color_msg)

        except Exception as e:
            self.get_logger().error(f"Error in stereo processing: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoDisparityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()