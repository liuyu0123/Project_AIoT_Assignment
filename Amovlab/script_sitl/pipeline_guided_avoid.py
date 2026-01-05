from pymavlink import mavutil
import time
import math
import numpy as np

# ================== MAVLink 连接 ==================
con = mavutil.mavlink_connection('udp:10.16.44.241:14550')
con.wait_heartbeat()
print("✔ 飞控心跳正常")

# ================== 全局障碍物 ==================
OBSTACLES_WGS = [
    (30.2492144, 120.1525663, 5.0),  # (lat, lon, radius_m)
]
print("✔ 障碍物：", OBSTACLES_WGS)

# ================== 避障参数 ==================
SAFE_DIST = 8.0        # m
TURN_ANGLE_DEG = 30.0
OBSTACLE_RANGES_M = [999.0] * 72


# =================================================
#                 工具函数
# =================================================
def set_param(name, value):
    con.mav.param_set_send(
        con.target_system,
        con.target_component,
        name.encode(),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)


def set_mode_guided():
    con.mav.command_long_send(
        con.target_system,
        con.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        15, 0, 0, 0, 0, 0
    )
    print("✔ 模式切换为 GUIDED")
    time.sleep(1)


def arm_vehicle():
    con.mav.command_long_send(
        con.target_system,
        con.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("✔ 解锁指令已发送")
    time.sleep(2)


# =================================================
#              障碍物 → BODY 坐标
# =================================================
def wgs_to_body(lat_s, lon_s, hdg_s, lat_o, lon_o):
    dlat = lat_o - lat_s
    dlon = lon_o - lon_s
    north = dlat * 111320
    east = dlon * 111320 * math.cos(math.radians(lat_s))
    hdg = math.radians(hdg_s)
    fwd = north * math.cos(hdg) + east * math.sin(hdg)
    right = -north * math.sin(hdg) + east * math.cos(hdg)
    return fwd, right


def gen_distance_array():
    N = 72
    angles = np.linspace(0, 360, N, endpoint=False)
    dist_cm = [65535] * N

    msg = con.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if not msg:
        return dist_cm

    lat = msg.lat * 1e-7
    lon = msg.lon * 1e-7
    hdg = msg.hdg * 1e-2

    for lat_o, lon_o, r in OBSTACLES_WGS:
        f, rgt = wgs_to_body(lat, lon, hdg, lat_o, lon_o)
        rng = math.hypot(f, rgt) - r
        if rng <= 0:
            rng = 0.05
        ang = math.degrees(math.atan2(rgt, f)) % 360
        width = max(10.0, math.degrees(math.atan2(r, rng)) * 2)
        for i, a in enumerate(angles):
            if min(abs(a-ang), 360-abs(a-ang)) <= width:
                dist_cm[i] = min(dist_cm[i], int(rng * 100))
    return dist_cm


def send_obstacle():
    global OBSTACLE_RANGES_M
    arr = gen_distance_array()
    OBSTACLE_RANGES_M = [
        d/100 if 0 < d < 65535 else 999 for d in arr
    ]
    con.mav.obstacle_distance_send(
        int(time.time()*1e6),
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        arr, 5, 0, 3000, 5.0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD
    )


# =================================================
#              控制函数（关键）
# =================================================
def stop_boat():
    con.mav.set_position_target_local_ned_send(
        0, con.target_system, con.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b110111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def turn_right():
    con.mav.set_attitude_target_send(
        0, con.target_system, con.target_component,
        0b00000100,
        [1, 0, 0, 0],
        0, 0, math.radians(TURN_ANGLE_DEG),
        0.3
    )


def go_to_target(lat, lon):
    con.mav.set_position_target_global_int_send(
        0, con.target_system, con.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        0b0000111111111000,
        int(lat*1e7), int(lon*1e7), 0,
        0, 0, 0, 0, 0, 0, 0, 0
    )


# =================================================
#                 主程序
# =================================================
def main():
    set_param('FRAME_CLASS', 2)
    set_param('ARMING_CHECK', 0)

    arm_vehicle()
    set_mode_guided()

    # 目标航点（直线穿过障碍）
    targets = [
        (30.2497209, 120.1523852),
        (30.2487078, 120.1527473),
    ]

    for lat, lon in targets:
        print(f"▶ 前往目标点 {lat:.7f}, {lon:.7f}")
        start = time.time()
        while time.time() - start < 25:
            send_obstacle()

            front = min(OBSTACLE_RANGES_M[34:38])
            if front < SAFE_DIST:
                print(f"⚠ 障碍 {front:.2f} m → 停船 + 右转")
                stop_boat()
                turn_right()
            else:
                go_to_target(lat, lon)

            time.sleep(0.1)

    print("✔ 任务结束")


if __name__ == "__main__":
    main()
