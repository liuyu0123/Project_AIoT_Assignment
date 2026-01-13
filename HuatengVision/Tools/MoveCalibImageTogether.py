#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把多个 Data*_512/left|right/*.png 复制到 OUTPUT_DIR/left|right/
新文件名格式：Data*_512_xxx.png
"""

import shutil
from pathlib import Path

# 1. 需要扫描的顶层目录列表
SRC_DIRS = [
    Path(r"D:\Files\Data\StereoCamera\ImageStereo\Calib\Data4_512"),
    Path(r"D:\Files\Data\StereoCamera\ImageStereo\Calib\Data5_512"),
    Path(r"D:\Files\Data\StereoCamera\ImageStereo\Calib\Data6_512"),
]

# 2. 结果保存目录
OUTPUT_DIR = Path(r"D:\Files\Data\StereoCamera\ImageStereo\Calib\Data4To6")   # 改成你想要的绝对/相对路径

# 3. 真正干活
for src_root in SRC_DIRS:
    for side in ('left', 'right'):
        src_side = src_root / side
        if not src_side.is_dir():
            continue

        dst_side = OUTPUT_DIR / side
        dst_side.mkdir(parents=True, exist_ok=True)

        for img_file in src_side.glob('*.png'):
            new_name = f'{src_root.name}_{img_file.name}'
            dst_file = dst_side / new_name
            shutil.copy2(img_file, dst_file)   # copy2 保留元数据
            print(f'copied  {img_file}  ->  {dst_file}')

print('全部复制完成！')