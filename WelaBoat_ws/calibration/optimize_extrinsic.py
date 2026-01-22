import numpy as np
import json
from scipy.optimize import least_squares
import cv2

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

P_lidar = np.array(json.load(open("cloud_points.json"))).T
p_img = np.array(json.load(open("image_points.json"))).T

def project(params, P):
    rvec = params[:3]
    tvec = params[3:].reshape(3,1)
    R, _ = cv2.Rodrigues(rvec)
    Pc = R @ P + tvec
    uv = K @ Pc
    uv = uv[:2] / uv[2]
    return uv

def error(params):
    uv = project(params, P_lidar)
    return (uv - p_img).flatten()

params0 = np.zeros(6)
res = least_squares(error, params0, loss='huber')
print("Optimized params:", res.x)
