#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把双目拼接视频解包成左右图片
"""
import cv2
import os
import sys
from pathlib import Path

# ========== 只用改这里 ==========
VIDEO_PATH   = "/home/pi/GitProject/LIUYU/HuatengVision/capture_video/stereo_videos/stereo_20260106_111942.mp4"  # 你的视频
OUTPUT_DIR   = "/home/pi/GitProject/LIUYU/HuatengVision/capture_video/stereo_videos_split"                       # 输出根目录
video_name = Path(VIDEO_PATH).stem          # 得到视频名称
OUTPUT_DIR   = os.path.join(OUTPUT_DIR, video_name)

# ================================

def split_video(video_path, out_root):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("打不开视频：", video_path)
        return

    w_tot = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps   = cap.get(cv2.CAP_PROP_FPS)
    n_tot = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w_single = w_tot // 2

    print(f"视频信息: {w_single}x{h}  {fps:.2f} fps  共 {n_tot} 帧")

    left_dir  = os.path.join(out_root, "left")
    right_dir = os.path.join(out_root, "right")
    os.makedirs(left_dir, exist_ok=True)
    os.makedirs(right_dir, exist_ok=True)

    idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        left  = frame[:, :w_single, :]
        right = frame[:, w_single:, :]

        # 文件名 6 位，左补零
        name = f"{idx:06d}.png"
        cv2.imwrite(os.path.join(left_dir,  name), left)
        cv2.imwrite(os.path.join(right_dir, name), right)

        if idx % 100 == 0:
            print(f"  已拆 {idx}/{n_tot} 帧")
        idx += 1

    cap.release()
    print(f"全部拆完！共 {idx} 帧，结果在 {out_root}")

if __name__ == "__main__":
    split_video(VIDEO_PATH, OUTPUT_DIR)