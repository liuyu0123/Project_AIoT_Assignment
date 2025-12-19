# author: young
import cv2
import numpy as np
import camera_configs

img = cv2.imread('70cm.png')

img_left = img[0:480, 0:640]
img_right = img[0:480, 640:1280]

img_left_rectified = cv2.remap(img_left, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
img_right_rectified = cv2.remap(img_right, camera_configs.right_map1, camera_configs.right_map2, cv2.INTER_LINEAR)
concat = cv2.hconcat([img_left_rectified, img_right_rectified])


i = 0
while (i < 480):
    cv2.line(img, (0, i), (1279, i), (0, 255, 0))
    cv2.line(concat, (0, i), (1279, i), (0, 255, 0))
    i = i + 36

cv2.imshow('original', img)
cv2.imshow('rectified', concat)

cv2.waitKey(0)


