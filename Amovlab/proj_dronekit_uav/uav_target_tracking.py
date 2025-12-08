from collections import deque
import numpy as np
import cv2
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
import time
from pymavlink import mavutil

pts = deque(maxlen=20)

hsv_lower = np.array([130,150,150])
hsv_upper = np.array([200,255,255])

ip_controller = "192.168.1.100:14550"
vehicle = connect(ip_controller, wait_ready=True)
vehicle.airspeed = 3
vehicle.groundspeed = 3

class Velocity():
    def set_velocity_body(self, vehicle, vx, vy, vz):
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,0,0,mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0,0,0,vx,vy,vz,0,0,0,0,0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.5)

def moving(ax, ay):
    vs2 = Velocity();
    if ax < 240:
        v2.set_velocity_body(vehicle, 0.5, 0, 0)
        print("Forward")
    elif ax > 240:
        v2.set_velocity_body(vehicle, -0.5, 0, 0)
        print("Back")
    if ay > 240:
        v2.set_velocity_body(vehicle, 0, -0.5, 0)
        print("Left")
    elif ay < 240:
        v2.set_velocity_body(vehicle, 0, 0.5, 0)
        print("Right")


def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            break
        time.sleep(1)


def mapObjectPosition(x, y):
    print("[INFO] Object Center coordenates at X0 = {0} and Y0 = {1}".format(x, y))


arm_and_takeoff(5)
time.sleep(2)

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
        print("Cannot receive frame (stream end?). Exiting ...")
        break

    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv_img, hsv_lower, hsv_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)
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
vehicle.close()