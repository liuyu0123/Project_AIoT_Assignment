from pymavlink import mavutil
import math, time, cv2

try:
    port = glob.glob('/dev/serial/by-id/usb-ArduPilot_fmuv3_*-if00')[0]  # 取第一个匹配
    print("检测到串口设备:",port)
except IndexError:
    raise SystemExit('找不到飞控串口，确认是否已插上')

# con = mavutil.mavlink_connection('/dev/serial0', baud=115200)
# con = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
con = mavutil.mavlink_connection(port, baud=115200)
con.wait_heartbeat()

# 相机参数
fx = 600
real_width = 0.5        # 人肩宽 0.5 m
angle_res = 10          # 36 槽
dist_arr = [65535]*36


# 把 36 点补成 72 点再发
def send_arr_36(arr36):
    arr72 = arr36 + [65535] * (72 - len(arr36))   # ★ 关键补长
    con.mav.obstacle_distance_send(
        int(time.time() * 1e6),
        1,                      # sensor_type LASER
        arr72,                  # ★ 必须 72 个 uint16
        angle_res,              # 角度步长 10°
        70, 2000,               # min/max cm
        angle_res * 0.01,       # increment_f (rad)  ★ 单位 0.01 rad
        0,                      # angle_offset (rad)
        12)                     # MAV_DISTANCE_SENSOR_LASER
    # ★ 打印还是用 36 点的索引，方便调试
    best_i = next((i for i, d in enumerate(arr36) if d != 65535), 0)
    print("sent dist=%d cm @ idx=%d" % (arr36[best_i], best_i))


def send_arr(arr):
    # arr72 = arr36 + [65535] * (72 - len(arr36))
    con.mav.obstacle_distance_send(
        int(time.time()*1e6),
        1,                      # sensor_type LASER
        arr, angle_res,
        70, 2000,               # min/max cm
        1.57, -1,               # 从 +90° 逆时针扫
        12)                     # MAV_DISTANCE_SENSOR_LASER
    print("sent dist=%d cm @ idx=%d" % (arr[best_i], best_i))


# 假视觉循环（替换成你的深度图）
cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    h, w = frame.shape[:2]
    # ---- 这里用模拟值：目标在画面中心，宽 80 px ----
    px_w = 80
    px_c = w//2
    d_cm = int(real_width * fx / px_w * 100)
    angle = math.atan2(px_c - w//2, fx) * 180 / math.pi
    idx = int((angle + 180) / angle_res) % 36
    dist_arr = [65535]*36
    dist_arr[idx] = min(d_cm, 2000)
    best_i = idx
    # ---------------------------------------------
    # send_arr(dist_arr)
    send_arr_36(dist_arr)

    time.sleep(0.1)