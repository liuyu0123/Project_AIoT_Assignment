import cv2
import numpy as np
import pipeline as camera_configs
import os
from pathlib import Path

# 数据集路径下应包含 left 和 right 两个文件夹
Data_DIR = r"D:\Files\Data\StereoCamera\ImageStereo\RiverImage\past"
# Data_DIR = r"D:\Files\Data\StereoUSV\test\stereo_20260108_160756_expose300"
left_dir = os.path.join(Data_DIR, "left")
right_dir = os.path.join(Data_DIR, "right")

# 检查路径是否异常
if not os.path.exists(left_dir) or not os.path.exists(right_dir):
    print("检查数据集文件夹路径下是否包含left和right文件夹")
    exit()

# 创建输出文件夹
# OUTPUT_DIR = Data_DIR + "_rectify"
left_out_dir = os.path.join(Data_DIR, "left_rectify")
right_out_dir = os.path.join(Data_DIR, "right_rectify")
# os.mkdir(OUTPUT_DIR)
if not os.path.exists(left_out_dir):
    os.mkdir(left_out_dir)
if not os.path.exists(right_out_dir):
    os.mkdir(right_out_dir)

# 遍历数据集下所有图片
for pic in Path(left_dir).rglob('*'):
    if pic.suffix in {'.jpg', '.png'}:
        # print(f"正在处理 {pic}")
        right_img_path = pic.parent.parent / "right" / pic.name
        if not os.path.exists(right_img_path): continue
        img_left = cv2.imread(str(pic))
        img_right = cv2.imread(str(right_img_path))
        # cv2.imshow("img_left", img_left)
        # cv2.imshow("img_right", img_right)
        # cv2.waitKey(0)

        img_left_rectified = cv2.remap(img_left, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
        img_right_rectified = cv2.remap(img_right, camera_configs.right_map1, camera_configs.right_map2, cv2.INTER_LINEAR)
        # imgL = cv2.cvtColor(img_left_rectified, cv2.COLOR_BGR2GRAY)
        # imgR = cv2.cvtColor(img_right_rectified, cv2.COLOR_BGR2GRAY)

        # 保存校正图片
        cv2.imwrite(os.path.join(left_out_dir, pic.name), img_left_rectified)
        cv2.imwrite(os.path.join(right_out_dir, pic.name), img_right_rectified)
