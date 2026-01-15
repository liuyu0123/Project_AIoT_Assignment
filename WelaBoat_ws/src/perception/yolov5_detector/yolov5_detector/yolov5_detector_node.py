#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import numpy as np
import os
import sys

# 添加 yolov5 路径（其实相对导入已足够，但保险起见）
FILE = os.path.dirname(os.path.abspath(__file__))
if FILE not in sys.path:
    sys.path.insert(0, FILE)

from .yolov5.models.common import DetectMultiBackend
from .yolov5.utils.general import non_max_suppression, scale_boxes
from .yolov5.utils.augmentations import letterbox


class YoloV5DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov5_detector')
        # 参数
        self.declare_parameter('model_path', '')
        model_path = self.get_parameter('model_path').value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f"Model path invalid: {model_path}")
            raise FileNotFoundError("YOLOv5 model not found!")

        self.declare_parameter('input_topic', '/camera/left/image_rect')
        input_topic = self.get_parameter('input_topic').value

        # 初始化模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = DetectMultiBackend(model_path, device=self.device)
        self.stride = int(self.model.stride)
        self.names = self.model.names
        self.conf_thres = 0.5
        self.iou_thres = 0.45

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            input_topic,  # 订阅左相机校正图
            self.image_callback,
            10
        )
        self.get_logger().info("YOLOv5 detector node started.")

    def image_callback(self, msg):
        try:
            # 转 OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            im0 = cv_image.copy()

            # 预处理：letterbox + 归一化
            img = letterbox(im0, stride=self.stride)[0]
            img = img.transpose((2, 0, 1))  # HWC to CHW
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device).float() / 255.0
            img = img[None]  # Add batch dim

            # 推理
            pred = self.model(img, augment=False, visualize=False)
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, None, False, max_det=100)

            # 这里先只打印检测结果（后续可发布 Detection2DArray）
            for det in pred[0]:
                if len(det):
                    det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], im0.shape).round()
                    for *xyxy, conf, cls in det:
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        self.get_logger().info(f"Detected: {label}")

            # TODO: 可视化或发布结果

        except Exception as e:
            self.get_logger().error(f"Error in detection: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloV5DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()