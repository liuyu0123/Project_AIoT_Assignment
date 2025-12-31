from pymavlink import mavutil
import time, glob, math
import numpy as np

con = mavutil.mavlink_connection('udp:10.16.44.241:14551')
con.wait_heartbeat()
print("飞控心跳正常")

# 世界坐标障碍物
OBSTACLES_WGS = [
    (30.2491416, 120.1524201, 4.0),   # (lat, lon, radius_m)
    (30.2489285, 120.1520070, 4.0),
]
print('=== 全局级 OBSTACLES_WGS =', OBSTACLES_WGS)



def param_get(param_id, timeout=3):
    """返回 (value, param_type)  失败返回 (None, None)"""
    con.mav.param_request_read_send(
        con.target_system,
        con.target_component,
        param_id.encode(),
        -1)                     # -1 表示按名字查
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = con.recv_match(type='PARAM_VALUE', blocking=False)
        if msg and msg.param_id == param_id:
            return msg.param_value, msg.param_type
    return None, None


def param_set(param_id, value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    """先 SET 再读回，确认生效"""
    con.mav.param_set_send(
        con.target_system,
        con.target_component,
        param_id.encode(),
        value,
        param_type)
    # 等飞控回播
    time.sleep(0.3)
    read_back, _ = param_get(param_id)
    if read_back is not None and abs(read_back - value) < 1e-5:
        print(f'{param_id} 已设成 {value}')
        return True
    else:
        print(f'{param_id} 写入失败（回读 {read_back}）')
        return False




def check_gps(gps_check=True):
    # 1. 解锁前检查
    if gps_check:
        while True:
            print("等待GPS模块准备就绪...")
            msg = con.recv_match(type='SYS_STATUS', blocking=True)
            if msg.onboard_control_sensors_health & 0x04:  # GPS OK
                break
            time.sleep(1)
        print("GPS模块已就绪")
    else:
        print("跳过GPS检查")


def vehicle_arming():
    """
    arm and then takeoff to aTargetAltitude in meters
    """
    # 1. 发送解锁指令
    print("准备解锁...")
    arming = 1
    con.mav.command_long_send(
        con.target_system,
        con.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,          # confirmation
        arming,     # 1 表示解锁
        0, 0, 0, 0, 0, 0)  

    # 2. 等 ACK，超时 3 秒
    ack = con.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    time.sleep(1)

    if ack is None:
        print("超时：没收到 COMMAND_ACK，不确定解锁结果")
    else:
        if ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("解锁成功！")
            elif ack.result == mavutil.mavlink.MAV_RESULT_FAILED:
                print("解锁失败，飞控拒绝")
            elif ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                print("暂时拒绝，常见原因：没通过预检、GPS 未定位、姿态误差大等")
            else:
                print("其他结果码：", ack.result)
        else:
            print("收到 ACK 但不是针对解锁指令")

    # 3. 再读一次心跳，顺便看 base_mode 的 ARM 标志
    msg = con.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print("心跳显示 armed =", armed)


    # 再读心跳，看 armed 标志是否保持
    msg = con.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print("二次心跳 armed =", armed)

    print("---- 连续 3 s 监控 armed 变化 ----")
    t0 = time.time()
    while time.time() - t0 < 3:
        hb = con.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            print(f"{time.time()-t0:.2f}s  armed={armed}")

        st = con.recv_match(type='STATUSTEXT', blocking=False)
        if st:
            txt = st.text if isinstance(st.text, str) else st.text.decode('utf-8', 'ignore')
            print(txt)
            print(f"{time.time()-t0:.2f}s  {txt}")
        time.sleep(0.05)


def set_mode(mode_id=15):
    con.mav.command_long_send(
            con.target_system, 
            con.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
            mode_id,                                            # custom_mode = GUIDED
            0,0,0,0,0)
    print("飞控模式切换为GUIDED...")
    ack_msg = con.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("模式切换成功！")
    else:
        print("模式切换失败，请检查飞控状态（如是否已解锁、模式是否支持）。")


def vehicle_takeoff(alt=3):
    # 起飞 3 m
    con.mav.command_long_send(
        con.target_system, con.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)  # param7=alt
    print("Takeoff 3 m sent")


def running2position(conn, speed_ms=3, target_lat=None, target_lon=None):
    """
    把船切到 Guided，并发一个“带线速度约束”的目标点。
    若不给目标点，就默认“当前船头方向 20 m 处”。
    """
    # 1. 确保 Guided 模式
    # set_mode_guided(conn)

    # 2. 取当前位置、航向
    msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    lat, lon = msg.lat * 1e-7, msg.lon * 1e-7
    heading_rad = msg.hdg * 1e-2 * math.pi / 180.0

    # 3. 如果没给目标，就往前 20 m 生成一个
    if target_lat is None or target_lon is None:
        # 简单平面近似：1 deg ≈ 111320 m
        dN = 20 * math.cos(heading_rad)
        dE = 20 * math.sin(heading_rad)
        target_lat = lat + dN / 111320.0
        target_lon = lon + dE / (111320.0 * math.cos(lat * math.pi / 180))

    # 4. 打包 SET_POSITION_TARGET_GLOBAL_INT
    type_mask = (0b0000111111111000 & ~0b0000000000001000)  # 允许 vx vy vz
    conn.mav.set_position_target_global_int_send(
        0,                       # time_boot_ms
        conn.target_system,
        conn.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
        type_mask,
        int(target_lat * 1e7),
        int(target_lon * 1e7),
        0,                       # alt
        speed_ms,                # vx  (m/s, NED 帧，正北)
        0,                       # vy
        0,                       # vz
        0, 0, 0,                 # afx/y/z
        0, 0                     # yaw, yaw rate
    )
    print(f"Guided → 目标点 {target_lat:.7f},{target_lon:.7f}  巡航速度 {speed_ms} m/s")




def wgs_to_body(lat_ship, lon_ship, hdg_ship,
                lat_obs, lon_obs):
    """返回 (forward_m, right_m) 船体坐标"""
    # 简单平面近似
    dLat = lat_obs - lat_ship
    dLon = lon_obs - lon_ship
    north = dLat * 111320.0
    east  = dLon * 111320.0 * math.cos(math.radians(lat_ship))
    # 旋转到船体 FRD
    hdg = math.radians(hdg_ship)
    f =  north * math.cos(hdg) + east * math.sin(hdg)
    r = -north * math.sin(hdg) + east * math.cos(hdg)
    return f, r


def gen_distance_array(master):
    print('=== 模块级 OBSTACLES_WGS =', OBSTACLES_WGS)
    """根据当前船位+障碍物→distances[72]"""
    N = 72
    angles = np.linspace(0, 360, N, endpoint=False)
    dist_cm = [65535] * N          # 65535 = 无检测
    # 取本船 GPS、航向
    g = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
    if not g:
        return dist_cm
    lat_s = g.lat * 1e-7
    lon_s = g.lon * 1e-7
    hdg_s = g.hdg * 1e-2           # deg

    for lat_o, lon_o, r_o in OBSTACLES_WGS:
        print('正在处理障碍物...')
        print('boat  lat=%.7f  lon=%.7f  hdg=%.1f°' % (lat_s, lon_s, hdg_s))
        print('obs   lat=%.7f  lon=%.7f  r=%.1f m' % (lat_o, lon_o, r_o))
        f, r = wgs_to_body(lat_s, lon_s, hdg_s, lat_o, lon_o)
        rng = math.hypot(f, r) - r_o
        if rng <= 0:               # 船已在障碍物内， clamp 1 cm
            rng = 0.01
        # if rng > 30:               # 30 m 外忽略
        #     continue
        # 计算遮挡角度范围
        ang_obs = math.degrees(math.atan2(r, f)) % 360
        # width   = 3 * math.degrees(math.atan2(r_o, rng))
        width   = max(10.0, math.degrees(math.atan2(r_o, rng)) * 2)
        for i, a in enumerate(angles):
            delta = min(abs(a - ang_obs), 360 - abs(a - ang_obs))
            if delta <= width:
                d_cm = int(rng * 100)
                # 取最近距离
                if dist_cm[i] == 65535 or d_cm < dist_cm[i]:
                    dist_cm[i] = d_cm
    return list(dist_cm)


def send_obstacle(master):
    arr = gen_distance_array(master)   # arr 已是 list
    print("正在发送障碍物信息...")
    print('sent OBSTACLE_DISTANCE:', len(arr), 'pts, min=%d max=%d cm' % (min(arr), max(arr)))
    master.mav.obstacle_distance_send(
        int(time.time() * 1000000),     # time_usec (μs)
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
        arr,                            # 长度 72 的 list
        5,                              # increment [deg]
        0,                              # min_distance
        30,                             # max_distance
        0.0,                            # increment_f
        0,                              # angle_offset
        mavutil.mavlink.MAV_FRAME_BODY_FRD
    )





def run():
    # 设置参数
    params = {
        'FRAME_CLASS': 2,   # 0=UNDEF 1=QUAD 2=HEX 3=OCTA 4=HELI 5=VTOL 6=BOAT
        'ARMING_CHECK': 0,  # 0=跳过全部自检
        'OA_TYPE': 2,       # Dijkstra
    }
    for name, val in params.items():
        param_set(name, float(val))

    # 检查 GPS
    check_gps(gps_check=True)
    # 解锁飞控
    vehicle_arming()
    # 切换模式
    set_mode(mode_id=15)
    # 发送障碍物信息
    send_obstacle(con)
    time.sleep(1.0)
    # 运动
    vehicle_trajectory = [
        30.2491416, 120.1524201,
        30.2489285, 120.1520070,
        30.2485485, 120.1522163,
        30.2487524, 120.1527795
    ]
    # running2position(con, speed_ms=3)
    loop_times = 1
    for _ in range(loop_times):
        for i in range(0, len(vehicle_trajectory), 2):
            running2position(con, speed_ms=3, target_lat=vehicle_trajectory[i], target_lon=vehicle_trajectory[i+1])
            time.sleep(10)


def run_new():
    # 1. 设置参数
    # BendyRuler 更适合处理实时的 OBSTACLE_DISTANCE 数据流进行动态避障
    params = {
        'FRAME_CLASS': 2,   # 2=HEX (根据您的实际载具调整)
        'ARMING_CHECK': 0,  # 0=跳过全部自检
        'OA_TYPE': 3,       # 修改为 1 (BendyRuler) - 这是正确的值
        'AVOID_MARGIN': 2,  # 避障安全边距(米)，替代之前的OA_LOOKAHEAD
        'OA_MARGIN_MAX': 5, # 最大避障边距(米)，替代之前的OA_MARGIN
    }
    for name, val in params.items():
        param_set(name, float(val))

    # 检查 GPS
    check_gps(gps_check=True)
    
    # 解锁飞控
    vehicle_arming()
    
    # 切换模式
    set_mode(mode_id=15)
    
    # 稍作等待
    time.sleep(1.0)

    # 轨迹点定义
    vehicle_trajectory = [
        30.2491416, 120.1524201,
        30.2489285, 120.1520070,
        30.2485485, 120.1522163,
        30.2487524, 120.1527795
    ]
    
    loop_times = 1
    
    # --- 开始循环移动 ---
    for _ in range(loop_times):
        for i in range(0, len(vehicle_trajectory), 2):
            target_lat = vehicle_trajectory[i]
            target_lon = vehicle_trajectory[i+1]
            
            # 发送目标点指令 (只发送一次，飞控会自动持续追踪)
            running2position(con, speed_ms=3, target_lat=target_lat, target_lon=target_lon)
            
            # 关键修改：在向下一个点移动的过程中，持续发送障碍物数据
            # 假设两点之间飞行/航行时间约为 10 秒
            start_time = time.time()
            while time.time() - start_time < 10:
                # 实时计算并发送障碍物距离数组
                send_obstacle(con)
                
                # 控制发送频率 (例如 10Hz)，避免阻塞消息队列
                time.sleep(0.1) 
                
            print(f"已到达航点 {i//2 + 1}")

    print("任务结束")



if __name__ == "__main__":
    run()

