#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import os
import time


class MonocularDistanceMeasurement:
    def __init__(self):
        # 窗口尺寸设置
        self.win_width  = 1920
        self.win_height = 1200
        self.mid_width  = self.win_width // 2
        self.mid_height = self.win_height // 2

        # 相机参数
        self.foc      = 2810.0   # 焦距（像素）
        self.real_wid = 11.69    # A4 纸实际宽度（英寸）

        # OpenCV 设置
        self.font  = cv2.FONT_HERSHEY_SIMPLEX
        self.w_ok  = 1

        # 初始化摄像头
        self.capture = None
        self.init_camera()

    # ---------------------------------------------------------
    # 自动尝试打开摄像头
    # ---------------------------------------------------------
    def init_camera(self):
        try:
            for camera_index in [0, 1, 2]:
                self.capture = cv2.VideoCapture(camera_index)
                if self.capture.isOpened():
                    print(f"成功连接摄像头 {camera_index}")
                    break
            else:
                print("警告：无法连接摄像头，将使用模拟模式")
                self.capture = None
                return

            # 设置分辨率
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,  self.win_width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.win_height)
        except Exception as e:
            print(f"摄像头初始化失败：{e}")
            self.capture = None

    # ---------------------------------------------------------
    # 无摄像头时生成演示画面
    # ---------------------------------------------------------
    def create_demo_frame(self):
        frame = np.full((self.win_height, self.win_width, 3), 50, dtype=np.uint8)
        rect_width, rect_height = 200, 150
        x = self.mid_width  - rect_width  // 2
        y = self.mid_height - rect_height // 2

        cv2.rectangle(frame, (x, y), (x + rect_width, y + rect_height),
                      (255, 255, 255), -1)
        cv2.rectangle(frame, (x, y), (x + rect_width, y + rect_height),
                      (0, 255, 0), 2)
        cv2.putText(frame, "Demo Mode - Simulated A4 Paper",
                    (x - 50, y - 20), self.font, 0.7, (255, 255, 255), 2)
        return frame

    # ---------------------------------------------------------
    # 图像处理：检测 A4 纸并计算距离
    # ---------------------------------------------------------
    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        binary = cv2.dilate(binary, kernel, iterations=2)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE,
                                       cv2.CHAIN_APPROX_SIMPLE)

        distance_cm = None
        for c in contours:
            if cv2.contourArea(c) < 2000:
                continue

            x, y, w, h = cv2.boundingRect(c)

            # 简单过滤
            if x > self.mid_width or y > self.mid_height:
                continue
            if (x + w) < self.mid_width or (y + h) < self.mid_height:
                continue
            if h > w or x == 0 or y == 0:
                continue
            if x + w >= self.win_width or y + h >= self.win_height:
                continue

            self.w_ok = w
            cv2.rectangle(frame, (x + 1, y + 1),
                          (x + self.w_ok - 1, y + h - 1),
                          (0, 255, 0), 2)

            dis_inch   = (self.real_wid * self.foc) / (self.w_ok - 2)
            distance_cm = dis_inch * 2.54
            break   # 只处理第一个符合的轮廓

        return frame, binary, distance_cm

    # ---------------------------------------------------------
    # 主循环
    # ---------------------------------------------------------
    def run(self):
        print("单目测距系统启动...")
        print("按 Esc 键退出程序，按空格键切换显示模式")
        show_binary = False

        while True:
            if self.capture and self.capture.isOpened():
                ret, frame = self.capture.read()
                if not ret:
                    print("无法读取摄像头数据")
                    break
            else:
                frame = self.create_demo_frame()

            processed, binary, distance_cm = self.process_frame(frame)

            if distance_cm:
                cv2.putText(processed, f"Distance: {distance_cm:.2f} cm",
                            (10, 30), self.font, 1.0, (0, 255, 0), 2)
            else:
                cv2.putText(processed, "No object detected",
                            (10, 30), self.font, 1.0, (0, 0, 255), 2)

            # 中心十字线
            cv2.putText(processed, "+",
                        (self.mid_width - 10, self.mid_height + 10),
                        self.font, 1.0, (0, 255, 0), 2)

            # 操作提示
            cv2.putText(processed, "Esc: Exit | SPACE: Toggle View",
                        (10, self.win_height - 20), self.font,
                        0.6, (255, 255, 255), 1)

            cv2.imshow("Monocular Distance Measurement",
                       binary if show_binary else processed)

            key = cv2.waitKey(40) & 0xFF
            if key == 27:       # Esc
                break
            elif key == 32:     # Space
                show_binary = not show_binary
                print(f"切换到 {'二值化' if show_binary else '原始'} 显示模式")

        # 清理
        if self.capture:
            self.capture.release()
        cv2.destroyAllWindows()
        print("程序已退出")


# ---------------------------------------------------------
# 直接运行本文件时的入口
# ---------------------------------------------------------
def main():
    print("=" * 50)
    print("单目视觉测距系统")
    print("基于相似三角形原理的距离测量")
    print("=" * 50)

    distance_system = MonocularDistanceMeasurement()
    distance_system.run()


if __name__ == "__main__":
    main()