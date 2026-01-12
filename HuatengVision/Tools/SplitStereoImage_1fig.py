#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将左右拼接的双目图像拆成左、右两张单目图
用法：修改 image_stereo 路径后运行即可
"""

import cv2
import os

# 1. 待拆分的拼接图路径
image_stereo = r"D:\Files\Data\StereoCamera\ImageStereo\Calib\Data4To6\test\sync_20260106_223748_072.png"         # ←←← 改成你的实际路径

# 2. 读图
img = cv2.imread(image_stereo)
if img is None:
    raise FileNotFoundError(f"无法读取图像：{image_stereo}")

# 3. 按宽度一分为二
h, w = img.shape[:2]
half = w // 2
left  = img[:, :half]
right = img[:, half:]

# 4. 构造输出文件名
base, ext = os.path.splitext(image_stereo)
left_name  = f"{base}_left{ext}"
right_name = f"{base}_right{ext}"

# 5. 保存
cv2.imwrite(left_name,  left)
cv2.imwrite(right_name, right)

print(f"已生成：\n  {left_name}\n  {right_name}")