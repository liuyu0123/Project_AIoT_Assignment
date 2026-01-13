# src/perception/stereo_disparity/stereo_disparity/disparity_node.py
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

        # ===== 声明参数 =====
        self.declare_parameter('minDisparity', 0)
        self.declare_parameter('numDisparities', 144)
        self.declare_parameter('blockSize', 11)
        self.declare_parameter('P1', 864)
        self.declare_parameter('P2', 3456)
        self.declare_parameter('disp12MaxDiff', 1)
        self.declare_parameter('uniquenessRatio', 15)
        self.declare_parameter('speckleWindowSize', 100)
        self.declare_parameter('speckleRange', 32)
        self.declare_parameter('mode', 1)
        self.declare_parameter('Q_matrix', [1.0]*16)  # 默认 4x4 单位矩阵展开

        # ===== 获取参数值 =====
        min_disp = self.get_parameter('minDisparity').value
        num_disp = self.get_parameter('numDisparities').value
        block_size = self.get_parameter('blockSize').value
        p1 = self.get_parameter('P1').value
        p2 = self.get_parameter('P2').value
        disp12_max_diff = self.get_parameter('disp12MaxDiff').value
        uniqueness_ratio = self.get_parameter('uniquenessRatio').value
        speckle_win_size = self.get_parameter('speckleWindowSize').value
        speckle_range = self.get_parameter('speckleRange').value
        mode = self.get_parameter('mode').value
        q_list = self.get_parameter('Q_matrix').value

        # 验证 numDisparities 是 16 的倍数
        if num_disp % 16 != 0:
            self.get_logger().warn(f"numDisparities ({num_disp}) is not divisible by 16. Adjusting to { (num_disp // 16) * 16 }")
            num_disp = (num_disp // 16) * 16

        # 初始化 SGBM
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            P1=p1,
            P2=p2,
            disp12MaxDiff=disp12_max_diff,
            uniquenessRatio=uniqueness_ratio,
            speckleWindowSize=speckle_win_size,
            speckleRange=speckle_range,
            mode=mode
        )

        # 重建 Q 矩阵
        self.Q = np.array(q_list, dtype=np.float32).reshape(4, 4)

        # 初始化 SGBM
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16 * 9,  # 必须是 16 的倍数
            blockSize=11,
            P1=8 * 3 * 11**2,
            P2=32 * 3 * 11**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )

        self.get_logger().info("Stereo disparity node started.")

    def stereo_callback(self, left_msg, right_msg):
        try:
            # 转换为 OpenCV 图像
            left_cv = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            right_cv = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')

            # 转灰度
            left_gray = cv2.cvtColor(left_cv, cv2.COLOR_BGR2GRAY)
            right_gray = cv2.cvtColor(right_cv, cv2.COLOR_BGR2GRAY)

            # 计算视差（输出为 16SC1，单位是 1/16 像素）
            disparity_16s = self.sgbm.compute(left_gray, right_gray)

            # 转为 float32（单位：像素）
            disparity = disparity_16s.astype(np.float32) / 16.0

            # 将无效视差设为 0（SGBM 输出中 -1 表示无效）
            disparity[disparity <= 0] = 0.0

            # 发布原始视差图（32FC1）
            disp_msg = self.bridge.cv2_to_imgmsg(disparity, encoding='32FC1')
            disp_msg.header = left_msg.header
            self.disparity_pub.publish(disp_msg)

            # 计算深度图（使用 Q 矩阵）
            if self.Q is not None:
                points_3d = cv2.reprojectImageTo3D(disparity, self.Q)
                depth = points_3d[:, :, 2]  # Z 通道
                depth[depth <= 0] = 0.0    # 无效深度设为 0

                depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
                depth_msg.header = left_msg.header
                self.depth_pub.publish(depth_msg)

            # 发布彩色视差图（用于 rviz 或 rqt_image_view 可视化）
            if self.disparity_color_pub.get_subscription_count() > 0:
                disp_norm = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX)
                disp_color = cv2.applyColorMap(disp_norm.astype(np.uint8), cv2.COLORMAP_JET)
                color_msg = self.bridge.cv2_to_imgmsg(disp_color, encoding='bgr8')
                color_msg.header = left_msg.header
                self.disparity_color_pub.publish(color_msg)

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