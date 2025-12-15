import cv2
import numpy as np
import argparse
import os
from utils import  YOLOWORLD

def main():
    parser = argparse.ArgumentParser(description='YOLO-World ONNX Inference')
    parser.add_argument('--model', type=str, default='../model/yolov8s-worldv2.q.onnx', help='Path to the YOLOv8 ONNX model')
    parser.add_argument('--image', type=str, default='../data/test.jpg', help='Path to the input image')
    parser.add_argument('--use-camera', action='store_true', help='Use camera as input')
    parser.add_argument('--camera-path', type=str, default='/dev/video20', help='Camera device path (default: /dev/video20)')
    parser.add_argument('--conf-threshold', type=float, default=0.2, help='Confidence threshold')    
    parser.add_argument('--iou-threshold', type=float, default=0.45, help='IoU threshold')
    parser.add_argument('--classes', nargs='+', type=str, default=['people'],help='Input class names: people car telephone')

    args = parser.parse_args()

    # Create detector 
    detector = YOLOWORLD(args.model,args.conf_threshold,args.iou_threshold)
    class_names = args.classes    
    detector.set_classes(class_names)

    if args.use_camera:        
        # 1. 解析参数（适配YOLO-World开放词汇检测）
        print(f"🎥 尝试打开摄像头: {args.camera_path}")
        print(f"🏷️  检测类别: {', '.join(class_names)}")
        
        # 2. 检查模型文件
        if not os.path.exists(args.model):
            print(f"❌ 模型文件不存在：{args.model}")
            return
        
        print(f"✅ YOLO-World模型加载成功: {args.model}")
        
        # 3. 优化的摄像头配置
        camera_path = args.camera_path
        print(f"📷 尝试打开摄像头: {camera_path}")
        
        # 关键：使用CAP_V4L2后端和优化参数
        # cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 固定分辨率
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # 固定分辨率
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 减少缓冲区延迟
        
        # 4. 检查摄像头是否成功打开
        if not cap.isOpened():
            print("❌ 无法打开摄像头设备")
            print("💡 可尝试的设备路径: /dev/video0, /dev/video1, /dev/video2, /dev/video3, /dev/video20, /dev/video21")
            return
        
        print("✅ 摄像头打开成功，开始YOLO-World开放词汇检测...")
        print(f"🎯 当前检测类别: {', '.join(class_names)}")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 无法从摄像头读取帧")
                break
                
            # 🌟 YOLO-World推理：传入自定义类别
            result_image = detector.infer(frame, class_names)

            # 显示结果
            cv2.imshow('YOLO-World Inference', result_image)

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
        result_image = detector.infer(image,class_names)

        # Save result image
        cv2.imwrite('result.jpg', result_image)
        print("Results saved to result.jpg")

if __name__ == "__main__":
    main()