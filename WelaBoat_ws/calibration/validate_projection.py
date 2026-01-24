import cv2
import numpy as np
import json

# ========= 配置 =========
# img = cv2.imread("/home/riba/GitProject/LIUYU/WelaBoat_ws/Data/data_calib3/left/0040.png")
img = cv2.imread("/home/riba/GitProject/LIUYU/WelaBoat_ws/Data/data_calib4_newRoundBoard/left/0022.png")


# json_file = '/home/riba/GitProject/LIUYU/WelaBoat_ws/src/drivers/camera_driver/camera_driver/utils/stereoParams_512pixel.json'
# with open(json_file, 'r') as f:
#     data = json.load(f)

# Extract camera matrices and distortion coefficients
# left_camera_matrix = np.array(data["Camera1"]["K"])
# K = left_camera_matrix

# 相机内参
# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0,  0,  1]])

# 标定结果（从 optimize_extrinsic.py 拷贝）
# Optimized_params = [ 0.790632,   -1.44847305,  1.63778087,  0.21818845, -0.15760476,  0.02509677]
# rvec = np.array([rx, ry, rz])
# tvec = np.array([[tx], [ty], [tz]])
# rvec = np.array([0.790632,   -1.44847305,  1.63778087])
# tvec = np.array([[0.21818845], [-0.15760476],  [0.02509677]])
ext = json.load(open("calib/extrinsic.json"))
rvec = np.array(ext["rvec"])
tvec = np.array(ext["tvec"]).reshape(3, 1)
K    = np.array(ext["camera_matrix"])

# 读取 LiDAR 点（可以是整帧，也可以是你点的点）
P_lidar = np.array(json.load(open("calib/cloud_points.json"))).T

# ========= 投影 =========
R, _ = cv2.Rodrigues(rvec)

for i in range(len(P_lidar.T)):
    P = P_lidar[:, i:i+1]
    Pc = R @ P + tvec

    if Pc[2] <= 0:
        continue

    uv = K @ Pc
    u = int(uv[0] / uv[2])
    v = int(uv[1] / uv[2])

    cv2.circle(img, (u, v), 4, (0, 0, 255), -1)

cv2.imshow("projection", img)
print("Press 's' to save the screenshot or any other key to exit.")
# cv2.waitKey(0)
key = cv2.waitKey(0) & 0xFF
if key == ord('s'):  # 按下 's' 键截图
    cv2.imwrite('calib/screenshot.png', img)
    print("截图已保存为 screenshot.png")
elif key == 27:  # ESC 键退出
    cv2.destroyAllWindows()
