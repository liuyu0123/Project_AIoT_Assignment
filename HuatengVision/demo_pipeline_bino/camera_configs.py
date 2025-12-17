# author: young
import cv2
import numpy as np

###########################################双目相机-内参#############################################3
# MATLAB内参矩阵(fx,fy,cx,cy):stereoParams.CameraParameters1.K
left_camera_matrix = np.array([[1.969e+03, 0, 1.045e+03], [0, 1.958e+03, 7.737e+02], [0, 0, 1]])
# MATLAB径向畸变(k1,k2,k3):stereoParams.CameraParameters1.RadialDistortion
# MATLAB切向畸变(p1,p2):stereoParams.CameraParameters1.TangentialDistortion
left_distortion = np.array([[-0.477, -0.125, -0.0020, -0.0096, 0.795]]) #k1,k2,p1,p2,k3

right_camera_matrix = np.array([[1.991e+03, 0, 1.115e+03], [0, 1.958e+03, 7.593e+02], [0, 0, 1]])
right_distortion = np.array([[-0.3763, 0.0819, 0.0049, 0.0088, 0.0493]])

###########################################双目相机-外参#############################################3
R = np.array([[0.9999, 0.0157, -0.0070],
              [-0.0157, 0.9999, 0.0017],
              [0.0070, -0.0016, 1.0000]])
# R = np.array([[0.9999, -0.0157, 0.0070],
#               [0.0157, 0.9999, -0.0017],
#               [-0.0070, 0.0016, 1.0000]])
T = np.array([-138.5193, 0.6695, -5.3973])

# size = (640, 480)  # open windows size
size = (1280, 720)   # open windows size
# size = (1024, 768)   # open windows size
# size = (320, 240)   # open windows size
# R1:左摄像机旋转矩阵, P1:左摄像机投影矩阵, Q:重投影矩阵
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

# 供外部脚本统一读取（单位：米 / 像素）
# b = abs(T[0]) * 1e-3          # 基线 m
# f = P1[0, 0]                  # 焦距 pixel
# cx = P1[0, 2]; cy = P1[1, 2]  # 主点

# 根据Bilibili教程
b = 1 / Q[3][2]
f = Q[2][3]
cx, cy = -Q[0][3], -Q[1][3]

if __name__ == '__main__':
    print(Q)

