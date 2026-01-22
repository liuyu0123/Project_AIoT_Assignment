import cv2
import json

points = []

def mouse_cb(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        points.append([x, y])
        print(f"Add point: {x}, {y}")

img = cv2.imread("/home/riba/GitProject/LIUYU/WelaBoat_ws/Data/data_calib3/left/0040.png")
cv2.imshow("image", img)
cv2.setMouseCallback("image", mouse_cb)
cv2.waitKey(0)

with open("image_points.json", "w") as f:
    json.dump(points, f)
