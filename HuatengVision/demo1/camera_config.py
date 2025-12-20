import numpy as np
import cv2

class StereoCameraConfig:
    def __init__(self):
        # 相机内参（需替换为标定后的实际参数）
        self.camera_matrix_left = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.camera_matrix_right = np.array([
            [800, 0, 320],
            [0, 800, 240],
            [0, 0, 1]
        ], dtype=np.float32)
        
        # 畸变系数
        self.dist_coeffs_left = np.array([-0.1, 0.01, 0, 0, 0], dtype=np.float32)
        self.dist_coeffs_right = np.array([-0.1, 0.01, 0, 0, 0], dtype=np.float32)
        
        # 外参（旋转和平移矩阵，需通过标定获得）
        self.R = np.eye(3, dtype=np.float32)  # 右相机相对于左相机的旋转
        self.T = np.array([-60, 0, 0], dtype=np.float32).reshape(3, 1)  # 基线距离60mm
        
        # 立体校正参数
        self.image_size = (640, 480)
        self.R1, self.R2, self.P1, self.P2, self.Q, _, _ = cv2.stereoRectify(
            self.camera_matrix_left, self.dist_coeffs_left,
            self.camera_matrix_right, self.dist_coeffs_right,
            self.image_size, self.R, self.T,
            alpha=0
        )
        
        # 计算映射矩阵
        self.map1x, self.map1y = cv2.initUndistortRectifyMap(
            self.camera_matrix_left, self.dist_coeffs_left,
            self.R1, self.P1, self.image_size, cv2.CV_32FC1
        )
        self.map2x, self.map2y = cv2.initUndistortRectifyMap(
            self.camera_matrix_right, self.dist_coeffs_right,
            self.R2, self.P2, self.image_size, cv2.CV_32FC1
        )