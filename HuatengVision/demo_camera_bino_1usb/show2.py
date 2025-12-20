#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import subprocess
import sys

def main():
    # 打开摄像头，索引1通常代表USB摄像头
    cap = cv2.VideoCapture(1)
    
    if not cap.isOpened():
        print("无法打开摄像头")
        sys.exit(-1)

    # 设置摄像头分辨率（可选）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    # 执行shell脚本（请修改为你的实际脚本路径）
    try:
        subprocess.run(["/home/camera.sh"], check=True)
        print("摄像头脚本执行成功")
    except FileNotFoundError:
        print("警告：未找到摄像头脚本 /home/camera.sh")
    except subprocess.CalledProcessError:
        print("警告：摄像头脚本执行失败")

    print("按 ESC 键退出程序")

    while True:
        # 读取一帧
        ret, frame = cap.read()
        
        if not ret or frame is None:
            print("无法读取摄像头数据")
            break

        # 调整图像大小为640x240
        resized_frame = cv2.resize(frame, (640, 240), interpolation=cv2.INTER_AREA)
        
        # 显示双目视图
        cv2.imshow("【双目视图】", resized_frame)
        
        # 分割为左右视图
        left_image = resized_frame[:, 0:320]  # 左视图：第0-319列
        right_image = resized_frame[:, 320:640]  # 右视图：第320-639列
        
        # 显示左右视图
        cv2.imshow("【左视图】", left_image)
        cv2.imshow("【右视图】", right_image)

        # 等待30ms，按ESC键退出
        key = cv2.waitKey(30) & 0xFF
        if key == 27:  # ESC键的ASCII码是27
            print("用户主动退出程序")
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    print("程序已退出")

if __name__ == "__main__":
    main()