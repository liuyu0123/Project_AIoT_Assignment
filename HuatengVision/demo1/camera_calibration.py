import cv2
import numpy as np
import os

def stereo_calibrate(left_images_dir, right_images_dir, chessboard_size=(9, 6), square_size=25):
    """双目相机标定"""
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # 准备棋盘格点
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    objpoints = []
    imgpoints_left = []
    imgpoints_right = []
    
    left_images = sorted(os.listdir(left_images_dir))
    right_images = sorted(os.listdir(right_images_dir))
    
    for left_img_name, right_img_name in zip(left_images, right_images):
        img_left = cv2.imread(os.path.join(left_images_dir, left_img_name))
        img_right = cv2.imread(os.path.join(right_images_dir, right_img_name))
        
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)
        
        ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)
        
        if ret_left and ret_right:
            objpoints.append(objp)
            
            corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
            corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)
            
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)
    
    # 分别标定单目相机
    ret_left, K_left, D_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
        objpoints, imgpoints_left, gray_left.shape[::-1], None, None
    )
    ret_right, K_right, D_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
        objpoints, imgpoints_right, gray_right.shape[::-1], None, None
    )
    
    #  stereoCalibrate
    flags = cv2.CALIB_FIX_INTRINSIC
    ret, K1, D1, K2, D2, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        K_left, D_left, K_right, D_right,
        gray_left.shape[::-1], criteria=criteria, flags=flags
    )
    
    print("标定完成！")
    print(f"左相机内参:\n{K1}")
    print(f"右相机内参:\n{K2}")
    print(f"旋转矩阵:\n{R}")
    print(f"平移向量:\n{T}")
    
    return K1, D1, K2, D2, R, T

if __name__ == '__main__':
    # 使用示例
    K1, D1, K2, D2, R, T = stereo_calibrate(
        'calibration_images/left',
        'calibration_images/right'
    )
    
    # 保存参数
    np.savez('camera_params.npz', K1=K1, D1=D1, K2=K2, D2=D2, R=R, T=T)