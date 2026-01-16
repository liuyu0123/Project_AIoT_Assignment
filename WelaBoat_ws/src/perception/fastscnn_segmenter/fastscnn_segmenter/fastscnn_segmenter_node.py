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

# === 添加 Fast-SCNN 项目根目录到 Python 路径 ===
FAST_SCNN_ROOT = os.path.expanduser("~/Fast-SCNN-pytorch")
if FAST_SCNN_ROOT not in sys.path:
    sys.path.insert(0, FAST_SCNN_ROOT)

try:
    from models.fast_scnn import FastSCNN
except ImportError as e:
    print(f"❌ Failed to import FastSCNN: {e}")
    print("💡 Make sure you have:")
    print("   git clone https://github.com/Tramac/Fast-SCNN-pytorch ~/Fast-SCNN-pytorch")
    print("   And that 'models/fast_scnn.py' exists.")
    sys.exit(1)

class FastSCNNSegmenterNode(Node):
    def __init__(self):
        super().__init__('fastscnn_segmenter')

        # 声明参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('input_topic', '/camera/left/image_rect')
        self.declare_parameter('output_mask_topic', '/fastscnn/segmentation')
        self.declare_parameter('output_color_topic', '/fastscnn/segmentation_color')
        self.declare_parameter('num_classes', 19)  # Cityscapes default

        model_path = self.get_parameter('model_path').value
        input_topic = self.get_parameter('input_topic').value
        output_mask_topic = self.get_parameter('output_mask_topic').value
        output_color_topic = self.get_parameter('output_color_topic').value
        self.num_classes = self.get_parameter('num_classes').value

        # 初始化模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = FastSCNN(num_classes=self.num_classes)
        
        if model_path and os.path.exists(model_path):
            self.get_logger().info(f'Loading model from {model_path}')
            checkpoint = torch.load(model_path, map_location=self.device)
            # Tramac 的权重直接保存 state_dict
            self.model.load_state_dict(checkpoint)
        else:
            self.get_logger().warn('No valid model path provided. Using random weights!')

        self.model.to(self.device).eval()

        # Cityscapes 19 类标准颜色映射
        self.color_map = self._get_cityscapes_colors()

        self.bridge = CvBridge()

        # 订阅与发布
        self.subscription = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.mask_publisher = self.create_publisher(Image, output_mask_topic, 10)
        self.color_publisher = self.create_publisher(Image, output_color_topic, 10)

        self.get_logger().info(
            f'✅ Fast-SCNN segmenter started.\n'
            f'  Input: {input_topic}\n'
            f'  Mask: {output_mask_topic} (mono8)\n'
            f'  Color: {output_color_topic} (bgr8)'
        )

    def _get_cityscapes_colors(self):
        return np.array([
            [128, 64, 128],   # road
            [244, 35, 232],   # sidewalk
            [70, 70, 70],     # building
            [102, 102, 156],  # wall
            [190, 153, 153],  # fence
            [153, 153, 153],  # pole
            [250, 170, 30],   # traffic light
            [220, 220, 0],    # traffic sign
            [107, 142, 35],   # vegetation
            [152, 251, 152],  # terrain
            [70, 130, 180],   # sky
            [220, 20, 60],    # person
            [255, 0, 0],      # rider
            [0, 0, 142],      # car
            [0, 0, 70],       # truck
            [0, 60, 100],     # bus
            [0, 80, 100],     # train
            [0, 0, 230],      # motorcycle
            [119, 11, 32],    # bicycle
            [0, 0, 0]         # void (index 19, if used)
        ], dtype=np.uint8)

    def image_callback(self, msg):
        try:
            # 转为 OpenCV BGR 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            orig_h, orig_w = cv_image.shape[:2]

            # 预处理：BGR → RGB → Resize → Tensor
            img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            img_resized = cv2.resize(img_rgb, (1024, 512))  # Fast-SCNN 默认输入尺寸
            img_tensor = torch.from_numpy(img_resized).float().permute(2, 0, 1) / 255.0
            img_tensor = img_tensor.unsqueeze(0).to(self.device)  # (1, 3, 512, 1024)

            # 推理
            with torch.no_grad():
                model_output = self.model(img_tensor)
                # 处理可能返回 tuple 的情况（aux loss enabled）
                if isinstance(model_output, tuple):
                    output = model_output[0]
                else:
                    output = model_output

                pred = torch.argmax(output, dim=1).squeeze(0)  # (512, 1024)

            # 缩放回原始图像尺寸（最近邻插值保持类别 ID 不变）
            pred_np = pred.cpu().numpy().astype(np.uint8)
            mask_full = cv2.resize(pred_np, (orig_w, orig_h), interpolation=cv2.INTER_NEAREST)

            # 发布原始语义分割图 (单通道 uint8)
            mask_msg = self.bridge.cv2_to_imgmsg(mask_full, encoding="mono8")
            mask_msg.header = msg.header
            self.mask_publisher.publish(mask_msg)

            # 生成彩色可视化图
            color_seg = self.color_map[mask_full]
            color_msg = self.bridge.cv2_to_imgmsg(color_seg, encoding="bgr8")
            color_msg.header = msg.header
            self.color_publisher.publish(color_msg)

        except Exception as e:
            self.get_logger().error(f'Error in segmentation: {str(e)}')  # 移除了 exc_info=True

def main(args=None):
    rclpy.init(args=args)
    node = FastSCNNSegmenterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()