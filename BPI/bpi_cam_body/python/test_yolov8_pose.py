import cv2
import numpy as np
import argparse
import os
from utils import  Yolov8Pose

def main():
    parser = argparse.ArgumentParser(description='YOLOv8-Pose ONNX Inference')
    parser.add_argument('--model', type=str, default='../model/yolov8n-pose.q.onnx', help='Path to the YOLOv8 ONNX model')
    parser.add_argument('--image', type=str, default='../data/test.jpg', help='Path to the input image')
    parser.add_argument('--use-camera', action='store_true', help='Use camera as input')
    parser.add_argument('--camera-path', type=str, default='/dev/video20', help='Camera device path (default: /dev/video20)')
    parser.add_argument('--conf-threshold', type=float, default=0.2, help='Confidence threshold')    
    parser.add_argument('--iou-threshold', type=float, default=0.45, help='IOU threshold')    
    args = parser.parse_args()

    # Create detector 
    detector = Yolov8Pose(args.model,args.conf_threshold,args.iou_threshold)

    if args.use_camera:        
        # 1. 解析参数（只保留核心功能，适配于摄像头，适配于设备）
        print(f"🎥 尝试打开摄像头: {args.camera_path}")
        
        # 2. 初始化检测器（简保模型路径正确）
        if not os.path.exists(args.model):
            print(f"❌ 模型文件不存在：{args.model}")
            return
        
        print(f"✅ 模型加载成功: {args.model}")
        
        # 3. 强制指定 /dev/video20 打开摄像头（核心适配）
        camera_path = args.camera_path
        print(f"📷 尝试打开摄像头: {camera_path}")
        
        # 关键：添加 OpenCV 摄像头参数配置（解决兼容性设备兼容性问题）
        cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)  # CAP_V4L2 是 Linux 专用视频驱动，适配设备兼容
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 固定分辨率，避免兼容性问题
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 固定分辨率，避免兼容性问题  
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 减少缓冲区，降低延迟
        
        # 4. 检查摄像头是否成功打开
        if not cap.isOpened():
            print("❌ 无法打开摄像头设备")
            print("💡 可尝试的设备路径: /dev/video0, /dev/video1, /dev/video2, /dev/video3, /dev/video20, /dev/video21")
            return
            
        print("✅ 摄像头打开成功，开始姿态检测...")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 无法从摄像头读取帧")
                break
                
            # 姿态检测推理
            result_image = detector.infer(frame)

            # 显示结果
            cv2.imshow('YOLOv8-Pose Inference', result_image)

            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
    else:
        if args.image is None:
            print("Please provide either an image path or use the --use-camera option.")
            return
        # Load image
        image = cv2.imread(args.image)
        if image is None:
            print(f"Failed to read image: {args.image}")
            return

        # Inference
        result_image = detector.infer(image)

        # Save result image
        cv2.imwrite('result.jpg', result_image)
        print("Results saved to result.jpg")

if __name__ == "__main__":
    main()