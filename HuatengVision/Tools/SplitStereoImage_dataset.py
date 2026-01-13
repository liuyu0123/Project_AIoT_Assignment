#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量拆分横向拼接双目图
用法：python batch_split_stereo.py 你的文件夹路径
"""

import cv2
import os
import sys
from pathlib import Path

SUPPORTED = {".png", ".jpg", ".jpeg", ".bmp", ".tiff", ".tif"}

def split_stereo(img_path: Path, out_left: Path, out_right: Path):
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"[WARN] 无法读取：{img_path}，跳过")
        return
    h, w = img.shape[:2]
    half = w // 2
    cv2.imwrite(str(out_left  / img_path.name), img[:, :half])
    cv2.imwrite(str(out_right / img_path.name), img[:, half:])

def main(root_dir: Path):
    left_dir  = root_dir / "left"
    right_dir = root_dir / "right"
    left_dir.mkdir(exist_ok=True)
    right_dir.mkdir(exist_ok=True)

    images = [p for p in root_dir.iterdir()
              if p.is_file() and p.suffix.lower() in SUPPORTED]
    if not images:
        print("目录内未找到支持的图片格式")
        return

    for img_path in images:
        split_stereo(img_path, left_dir, right_dir)

    print(f"全部处理完成！left={left_dir}  right={right_dir}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("用法: python batch_split_stereo.py <图片所在文件夹>")
        sys.exit(1)
    main(Path(sys.argv[1]).expanduser().resolve())