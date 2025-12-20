import cv2
import torch
import numpy as np
from camera_config import StereoCameraConfig
from dis_count import DepthEstimator
from pathlib import Path

class StereoYOLOv5Detector:
    def __init__(self, yolov5_path='yolov5', model_weights='yolov5s.pt', device=''):
        self.config = StereoCameraConfig()
        self.depth_estimator = DepthEstimator(self.config)
        
        # 加载YOLOv5模型
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = torch.hub.load(yolov5_path, 'custom', path=model_weights, source='local')
        self.model.conf = 0.25  # 置信度阈值
        self.model.iou = 0.45   # NMS阈值
        self.model.to(self.device)
        
        # 类别名称
        self.classes = self.model.names
    
    def detect_and_measure(self, left_img, right_img):
        """执行检测和测距"""
        # 计算深度图
        depth_map, left_rect = self.depth_estimator.get_depth_map(left_img, right_img)
        
        # YOLOv5检测（使用校正后的左图）
        results = self.model(left_rect)
        detections = results.xyxy[0]  # [x1, y1, x2, y2, conf, cls]
        
        # 结果列表
        measured_objects = []
        
        for det in detections:
            x1, y1, x2, y2, conf, cls = det.tolist()
            
            # 计算目标距离
            distance = self.depth_estimator.get_object_distance(depth_map, [x1, y1, x2, y2])
            
            if distance > 0:
                obj_info = {
                    'class': self.classes[int(cls)],
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'confidence': float(conf),
                    'distance': distance  # 单位：毫米
                }
                measured_objects.append(obj_info)
        
        return measured_objects, depth_map, left_rect
    
    def visualize(self, img, detections, depth_map=None):
        """可视化检测结果和距离信息"""
        vis_img = img.copy()
        
        for obj in detections:
            x1, y1, x2, y2 = obj['bbox']
            label = f"{obj['class']} {obj['confidence']:.2f}"
            distance = f"{obj['distance']:.2f}mm"
            
            # 绘制边界框
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 绘制标签和距离
            text_y = y1 - 10 if y1 > 20 else y1 + 20
            cv2.putText(vis_img, label, (x1, text_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(vis_img, distance, (x1, text_y + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        
        # 可视化深度图（可选）
        if depth_map is not None:
            depth_vis = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("Depth Map", depth_vis)
        
        return vis_img

def main():
    # 初始化检测器
    detector = StereoYOLOv5Detector()
    
    # 打开双目相机（双设备号）
    cap_left = cv2.VideoCapture(0)  # 左相机
    cap_right = cv2.VideoCapture(2)  # 右相机，根据实际情况调整
    
    if not cap_left.isOpened() or not cap_right.isOpened():
        print("无法打开相机")
        return
    
    # 设置分辨率
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    print("双目YOLOv5测距系统已启动...")
    
    while True:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()
        
        if not ret_left or not ret_right:
            print("读取帧失败")
            break
        
        # 执行检测和测距
        detections, depth_map, left_rect = detector.detect_and_measure(frame_left, frame_right)
        
        # 可视化结果
        result_img = detector.visualize(left_rect, detections, depth_map)
        
        # 显示结果
        cv2.imshow("Stereo YOLOv5 Detection", result_img)
        
        # 打印检测信息
        for obj in detections:
            print(f"类别: {obj['class']}, 距离: {obj['distance']:.2f}mm")
        
        # 按Q退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()