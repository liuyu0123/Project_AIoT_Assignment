import cv2
import numpy as np
import argparse
import os
from utils import Detection

def main():
    parser = argparse.ArgumentParser(description='YOLOv5-Face Detection')
    parser.add_argument('--model', type=str, default='../model/yolov5n-face_320_cut.q.onnx', help='Path to ONNX model')
    parser.add_argument('--image', type=str, default='../data/test.jpg', help='Path to input image')
    parser.add_argument('--use-camera', action='store_true', help='Use camera as input')
    parser.add_argument('--camera-path', type=str, default='/dev/video20', help='Camera device path (default: /dev/video20)')
    parser.add_argument('--conf-threshold', type=float, default=0.4, help='Confidence threshold')    
    parser.add_argument('--iou-threshold', type=float, default=0.5, help='IoU threshold for NMS')        
    args = parser.parse_args()

    # 检查模型文件是否存在        
    if not os.path.exists(args.model):
        print(f"❌ 模型文件不存在：{args.model}")
        return
        
    det = Detection(args.model, args.conf_threshold, args.iou_threshold)

    if args.use_camera:
        # 1. 优化的摄像头配置
        print(f"🎥 尝试打开摄像头: {args.camera_path}")
        print(f"✅ YOLOv5-Face模型加载成功: {args.model}")
        
        # 2. 使用优化的摄像头配置
        camera_path = args.camera_path
        print(f"📷 尝试打开摄像头: {camera_path}")
        
        # 关键：使用CAP_V4L2后端和优化参数
        cap = cv2.VideoCapture(camera_path, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 设置分辨率
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 减少缓冲区延迟
        
        # 3. 检查摄像头是否成功打开
        if not cap.isOpened():
            print("❌ 无法打开摄像头设备")
            print("💡 可尝试的设备路径: /dev/video0, /dev/video1, /dev/video2, /dev/video3, /dev/video20, /dev/video21")
            return
            
        print("✅ 摄像头打开成功，开始人脸检测...")
        
        # 4. 主循环：实时人脸检测
        while True:
            ret, frame = cap.read()
            if not ret:
                print("❌ 无法从摄像头读取帧")
                break
                
            # 复制帧用于推理（修复变量名错误）
            frame_copy = frame.copy()
            
            # 人脸检测推理
            boxes = det.infer(frame_copy)
            
            # 在原始帧上绘制检测结果
            for box in boxes[0]:                        
                cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                # 可选：添加置信度标签
                confidence = box[4] if len(box) > 4 else 0.0
                cv2.putText(frame, f'Face: {confidence:.2f}', (int(box[0]), int(box[1] - 10)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # 显示结果
            cv2.imshow('YOLOv5-Face Detection', frame)
            
            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()

    else:
        # 图片模式
        if not os.path.exists(args.image):
            print(f"❌ 图片文件不存在：{args.image}")
            return
            
        print(f"📷 加载图片: {args.image}")
        img = cv2.imread(args.image)
        if img is None:
            print(f"❌ 无法读取图片：{args.image}")
            return
            
        print("🔍 开始人脸检测...")
        boxes = det.infer(img)
        
        # 绘制检测结果
        face_count = 0
        for box in boxes[0]:                        
            cv2.rectangle(img, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
            # 添加置信度标签
            confidence = box[4] if len(box) > 4 else 0.0
            cv2.putText(img, f'Face: {confidence:.2f}', (int(box[0]), int(box[1] - 10)), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            face_count += 1

        # 保存结果
        cv2.imwrite("result.jpg", img)
        print(f"✅ 检测完成！发现 {face_count} 张人脸")
        print("📁 结果已保存到 result.jpg")
        

if __name__ == '__main__':
    main()