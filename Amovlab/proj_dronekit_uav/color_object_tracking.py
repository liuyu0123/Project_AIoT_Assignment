from collections import deque
import numpy as np
import cv2

pts = deque(maxlen=20)

hsv_lower = np.array([130,150,150])
hsv_upper = np.array([200,255,255])

def mapObjectPosition(x, y):
    print("[INFO] OBject Center coordenates at X0 = {0} and Y = {1}".format(x, y))

camera = cv2.VideoCapture(0)
camera.set(3,640)
camera.set(4,480)

if not camera.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = camera.read()
    ret, frame = camera.read()
    if not ret:
        print("Can not receive frame (stream end?). Exiting ...")
        break

    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0,255,255), 2)
            cv2.circle(frame, center, 5, (0,0,255), -1)
            mapObjectPosition(int(x), int(y))

    pts.appendleft(center)

    for i in range(1, len(pts)):
        if pts[i - 1] is None or pts[i] is None:
            continue
        thickness = int(np.sqrt(20/float(i+1))*2.5)
        cv2.line(frame, pts[i-1], pts[i], (0,0,255), thickness)

    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

camera.release()
cv2.destroyAllWindows()