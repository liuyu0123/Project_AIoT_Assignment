import cv2
import numpy as np
import torch

class DepthEstimator:
    def __init__(self, camera_config, min_disp=16, num_disp=128):
        self.camera_config = camera_config
        
        # 立体匹配参数（SGBM算法）
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=9,
            P1=8 * 3 * 9 ** 2,
            P2=32 * 3 * 9 ** 2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            preFilterCap=63,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
    
    def get_depth_map(self, left_img, right_img):
        """计算深度图"""
        # 图像校正
        left_rect = cv2.remap(
            left_img, self.camera_config.map1x, self.camera_config.map1y,
            cv2.INTER_LINEAR
        )
        right_rect = cv2.remap(
            right_img, self.camera_config.map2x, self.camera_config.map2y,
            cv2.INTER_LINEAR
        )
        
        # 转换为灰度图
        gray_left = cv2.cvtColor(left_rect, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_rect, cv2.COLOR_BGR2GRAY)
        
        # 计算视差图
        disparity = self.stereo.compute(gray_left, gray_right).astype(np.float32) / 16.0
        
        # 计算深度图 (depth = baseline * focal / disparity)
        baseline = np.linalg.norm(self.camera_config.T)
        focal = self.camera_config.camera_matrix_left[0, 0]
        disparity[disparity == 0] = 0.1  # 避免除零
        depth = baseline * focal / disparity
        
        return depth, left_rect
    
    def get_object_distance(self, depth_map, bbox):
        """计算目标边界框内的平均距离"""
        x_min, y_min, x_max, y_max = map(int, bbox)
        x_min = max(0, x_min)
        y_min = max(0, y_min)
        x_max = min(depth_map.shape[1], x_max)
        y_max = min(depth_map.shape[0], y_max)
        
        # 提取目标区域的深度值
        depth_roi = depth_map[y_min:y_max, x_min:x_max]
        
        # 过滤无效值和异常值
        valid_depth = depth_roi[(depth_roi > 0.1) & (depth_roi < 100)]
        
        if len(valid_depth) == 0:
            return -1
        
        # 使用中位数距离更鲁棒
        distance = np.median(valid_depth)
        return float(distance)