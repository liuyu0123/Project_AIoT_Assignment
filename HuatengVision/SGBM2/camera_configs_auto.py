import json
import numpy as np
import cv2


filename = r'D:\AIoT\HuatengVision\calibrate_camera_bino\stereoParams14cm.json'
size = (1280, 720)  # You can adjust this based on your needs
CamConf = dict()
CamConf['size'] = size

def load_camera_config(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    # Extract camera matrices and distortion coefficients
    left_camera_matrix = np.array(data["Camera1"]["K"])
    right_camera_matrix = np.array(data["Camera2"]["K"])
    
    left_distortion = np.array([data["Camera1"]["RadialDistortion"][0], 
                                data["Camera1"]["RadialDistortion"][1], 
                                0, 0, 0])  # Add p1, p2, k3 as 0 if not provided
    right_distortion = np.array([data["Camera2"]["RadialDistortion"][0], 
                                 data["Camera2"]["RadialDistortion"][1], 
                                 0, 0, 0])  # Add p1, p2, k3 as 0 if not provided

    # Extract rotation and translation matrices
    R = np.array(data["PoseCamera2"]["R"])
    T = np.array(data["PoseCamera2"]["Translation"])

    # Set the size of the images

    # Perform stereo rectification
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
        left_camera_matrix, left_distortion,
        right_camera_matrix, right_distortion, size, R, T)

    # Initialize undistort rectify maps
    left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

    # Calculate baseline, focal length, and principal point
    b = 1 / Q[3][2]
    f = Q[2][3]
    cx, cy = -Q[0][3], -Q[1][3]
    CamConf['Q'] = Q

    return left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, R, T, \
        left_map1, left_map2, right_map1, right_map2, b, f, cx, cy


# 读取参数并记录在字典中， 便于代码使用
left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, R, T, \
    left_map1, left_map2, right_map1, right_map2, b, f, cx, cy = load_camera_config(filename)
CamConf['left_map1'] = left_map1
CamConf['left_map2'] = left_map2
CamConf['right_map1'] = right_map1
CamConf['right_map2'] = right_map2
CamConf['b'] = b
CamConf['f'] = f
CamConf['cx'] = cx
CamConf['cy'] = cy


if __name__ == '__main__':
    print('Left Camera Matrix:\n', left_camera_matrix)
    print('Left Distortion:\n', left_distortion)
    print('Right Camera Matrix:\n', right_camera_matrix)
    print('Right Distortion:\n', right_distortion)
    print('Rotation Matrix R:\n', R)
    print('Translation Vector T:\n', T)
    print('Baseline b: ', b)
    print('Focal Length f: ', f)
    print('Principal Point (cx, cy): ', cx, cy)