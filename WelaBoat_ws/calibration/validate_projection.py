import cv2
import numpy as np
import json

# ========= 配置 =========
img = cv2.imread("image.png")

# 相机内参
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

# 标定结果（从 optimize_extrinsic.py 拷贝）
rvec = np.array([rx, ry, rz])
tvec = np.array([[tx], [ty], [tz]])

# 读取 LiDAR 点（可以是整帧，也可以是你点的点）
P_lidar = np.array(json.load(open("cloud_points.json"))).T

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
cv2.waitKey(0)
