#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把双目拼接视频解包成左右图片（批量版）
"""
import cv2
import os
import sys
from pathlib import Path

# ========== 只用改这里 ==========
ROOT_VIDEO_DIR = r"D:\Files\Data\StereoUSV\test"  # 你的视频根目录
OUTPUT_DIR     = ROOT_VIDEO_DIR  # 输出根目录
# ================================

# 常见视频后缀
VIDEO_EXTS = {'.mp4', '.avi', '.mov', '.mkv', '.flv', '.wmv', '.ts', '.m4v'}

def find_videos(root_dir):
    """返回 root_dir 下所有视频文件的绝对路径列表"""
    root_path = Path(root_dir).expanduser()
    if not root_path.is_dir():
        raise ValueError(f"输入路径不是目录: {root_dir}")
    videos = []
    for ext in VIDEO_EXTS:
        videos.extend(root_path.rglob(f'*{ext}'))
    return sorted(videos)

def split_video(video_path, out_root):
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print("打不开视频：", video_path)
        return

    w_tot = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h     = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps   = cap.get(cv2.CAP_PROP_FPS)
    n_tot = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w_single = w_tot // 2

    print(f"[{video_path.name}] 视频信息: {w_single}x{h}  {fps:.2f} fps  共 {n_tot} 帧")

    parent_dir = os.path.dirname(out_root)
    child_dir = os.path.basename(out_root)
    print(f"parent_dir: {parent_dir}")
    print(f"child_dir: {child_dir}")

    left_dir  = Path(parent_dir) / "left"  / child_dir
    right_dir = Path(parent_dir) / "right" / child_dir
    left_dir.mkdir(parents=True, exist_ok=True)
    right_dir.mkdir(parents=True, exist_ok=True)

    idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        left  = frame[:, :w_single, :]
        right = frame[:, w_single:, :]

        name = f"{idx:06d}.png"
        cv2.imwrite(str(left_dir  / name), left)
        cv2.imwrite(str(right_dir / name), right)

        if idx % 100 == 0:
            print(f"  已拆 {idx}/{n_tot} 帧")
        idx += 1

    cap.release()
    print(f"[{video_path.name}] 全部拆完！共 {idx} 帧，结果在 {out_root}")

def main():
    videos = find_videos(ROOT_VIDEO_DIR)
    if not videos:
        print("目录下未找到任何视频文件！")
        return

    print(f"共发现 {len(videos)} 个视频，开始批量处理…")
    for v in videos:
        out_sub = Path(OUTPUT_DIR) / v.stem
        split_video(v, out_sub)

if __name__ == "__main__":
    main()