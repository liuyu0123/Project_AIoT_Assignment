# author: young
import cv2
import numpy as np


left_camera_matrix = np.array([[961.7192, 0, 295.3539], [0, 963.1369, 294.4663], [0, 0, 1]])
left_distortion = np.array([[0.1351, 0.2322, 0.0017, -0.0023, 0]])

right_camera_matrix = np.array([[960.3725, 0, 300.5267], [0, 963.0802, 277.6427], [0, 0, 1]])
right_distortion = np.array([[0.0965, 1.0347, 0.0044, -0.0042, 0]])

R = np.array([[0.9996, -0.00055, 0.0289],
              [0.00059, 1, -0.0015],
              [-0.0289, 0.0015, 0.9996]])
T = np.array([-84.8645, -0.0376, 1.3472])

size = (640, 480)  # open windows size
# R1:左摄像机旋转矩阵, P1:左摄像机投影矩阵, Q:重投影矩阵
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

print(Q)


