#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import numpy as np
import os
import sys
import cv2
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose2D

# === 添加 YOLOv5 路径（请确保 ~/yolov5 是官方仓库）===
YOLOV5_PATH = os.path.expanduser("~/yolov5")
if YOLOV5_PATH not in sys.path:
    sys.path.insert(0, YOLOV5_PATH)

try:
    from utils.augmentations import letterbox
    from utils.general import non_max_suppression, scale_boxes
    from models.common import DetectMultiBackend
except ImportError as e:
    print(f"Failed to import YOLOv5 modules: {e}")
    print("Make sure ~/yolov5 exists and is the official Ultralytics repo.")
    sys.exit(1)

class YoloV5DetectorNode(Node):
    def __init__(self):
        super().__init__('yolov5_detector')

        # 声明参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_topic', '/camera/left/image_rect')
        self.declare_parameter('output_topic', '/yolov5/detections_image')
        self.declare_parameter('conf_thres', 0.5)
        self.declare_parameter('iou_thres', 0.45)

        model_path = self.get_parameter('model_path').value
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model path invalid or not provided: {model_path}')
            raise FileNotFoundError('YOLOv5 model not found!')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.conf_thres = self.get_parameter('conf_thres').value
        self.iou_thres = self.get_parameter('iou_thres').value

        # 初始化模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = DetectMultiBackend(model_path, device=self.device)
        self.stride = int(self.model.stride.max()) if hasattr(self.model.stride, 'max') else int(self.model.stride)
        self.names = self.model.names if hasattr(self.model, 'names') else [f'class{i}' for i in range(1000)]

        self.bridge = CvBridge()

        # 订阅与发布
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.publisher_ = self.create_publisher(Image, output_topic, 10)
        # 发布2D检测框
        self.detection_publisher_ = self.create_publisher(Detection2DArray, '/yolov5/bboxes', 10)

        self.get_logger().info(
            f'YOLOv5 detector started.\n'
            f'  Model: {model_path}\n'
            f'  Input: {input_topic}\n'
            f'  Output: {output_topic}'
        )

    def image_callback(self, msg):
        try:
            # 转为 OpenCV BGR 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            im0 = cv_image.copy()
            h0, w0 = im0.shape[:2]

            # 预处理：letterbox + 归一化
            img = letterbox(im0, new_shape=(640, 640), stride=self.stride, auto=True)[0]
            img = img.transpose((2, 0, 1))  # HWC to CHW
            img = np.ascontiguousarray(img)
            img = torch.from_numpy(img).to(self.device).float() / 255.0
            img = img.unsqueeze(0)  # Add batch dimension

            # 推理
            with torch.no_grad():
                pred = self.model(img, augment=False, visualize=False)
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, None, False, max_det=100)

            # 绘制结果
            det_img = im0.copy()
            det = pred[0]  # (n, 6) tensor: [x1, y1, x2, y2, conf, cls]

            # 创建 Detection2DArray 消息
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header  # 保持时间戳和 frame_id 一致

            if det is not None and len(det) > 0:
                # 确保是二维张量
                if det.dim() == 1:
                    det = det.unsqueeze(0)

                # 缩放坐标回原图尺寸
                det[:, :4] = scale_boxes(img.shape[2:], det[:, :4], det_img.shape).round()

                for detection in det:
                    x1, y1, x2, y2, conf, cls = detection.cpu().numpy()
                    x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                    cls = int(cls)

                    if conf < self.conf_thres:
                        continue

                    # 设置边界框中心、大小
                    center_x = (x1 + x2) / 2.0
                    center_y = (y1 + y2) / 2.0
                    width = x2 - x1
                    height = y2 - y1

                    # 构造单个 Detection2D
                    detection2d = Detection2D()
                    detection2d.bbox.center.position.x = float(center_x)
                    detection2d.bbox.center.position.y = float(center_y)
                    detection2d.bbox.center.theta = 0.0
                    detection2d.bbox.size_x = float(width)
                    detection2d.bbox.size_y = float(height)

                    # 设置类别和置信度
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = self.names[cls]
                    hypothesis.hypothesis.score = float(conf)
                    detection2d.results.append(hypothesis)
                    detections_msg.detections.append(detection2d)

                    # 准备将检测框绘制在原始图像上
                    label = f'{self.names[cls]} {conf:.2f}'
                    # 绘制框
                    cv2.rectangle(det_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    # 绘制标签
                    cv2.putText(det_img, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)



                    # self.get_logger().info(f'Detected: {label} at ({x1},{y1})-({x2},{y2})')

            # 发布结果图像
            out_msg = self.bridge.cv2_to_imgmsg(det_img, encoding="bgr8")
            out_msg.header = msg.header
            self.publisher_.publish(out_msg)

            # 发布2D检测框检测结果
            self.detection_publisher_.publish(detections_msg)

        except Exception as e:
            self.get_logger().error(f'Error in detection: {str(e)}')

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