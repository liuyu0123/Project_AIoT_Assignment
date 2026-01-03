from pymavlink import mavutil
import time, glob, math
import numpy as np

con = mavutil.mavlink_connection('udp:10.16.44.241:14550')
# con = mavutil.mavlink_connection('udpout:127.0.0.1:14550', source_system=1, source_component=1)
con.wait_heartbeat()
print("飞控心跳正常")

# 世界坐标障碍物(lat, lon, radius_m)
OBSTACLES_WGS = [
    (30.2492144, 120.1525663, 5.0),
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

def set_mode(mode_id):
    """
    Rover 切 AUTO 的标准做法：只给 base_mode 打 CUSTOM_MODE 标志，
    custom_mode 写 10，其余全 0。
    """
    con.mav.command_long_send(
        con.target_system,
        con.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,                                     # confirmation
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # base_mode
        mode_id,                                    # custom_mode = AUTO(10) GUIDED(15)
        0, 0, 0, 0, 0)                         # 参数 4~7 必须填 0

    print("飞控模式切换为ID=", mode_id)
    ack = con.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if not ack:
        print("超时：根本没收到 ACK")
        return False

    print(f"ACK 结果码: {ack.result}")
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("模式切换成功！")
        return True
    else:
        print("模式切换失败")
        return False

def send_waypoints(waypoints):
    """发送航点任务（waypoints格式：[(lat1, lon1), (lat2, lon2), ...]）"""
    # 清除现有任务 - 修正：添加target_system和target_component
    con.mav.mission_clear_all_send(
        con.target_system,
        con.target_component
    )
    time.sleep(1)
    
    # 发送航点数量
    con.mav.mission_count_send(
        con.target_system,
        con.target_component,
        len(waypoints)
    )
    time.sleep(1)
    
    # 逐个发送航点
    for i, (lat, lon) in enumerate(waypoints):
        con.mav.mission_item_send(
            con.target_system,
            con.target_component,
            i,  # 序号
            mavutil.mavlink.MAV_FRAME_GLOBAL_INT,  # 坐标系
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # 命令类型
            0,  # 当前点是否为任务终点（0=否，1=是）
            0,  # 自动继续（0=是，1=否）
            0, 0, 0, 0, 0,  # 参数1-5（保留）
            int(lat * 1e7),  # 纬度（*1e7）
            int(lon * 1e7)   # 经度（*1e7）
        )
        time.sleep(0.1)  # 避免消息阻塞
    
    # 启动任务（从第0个航点开始）
    con.mav.mission_set_current_send(
        con.target_system,
        con.target_component,
        0
    )
    print("航点任务已发送，飞控开始执行...")

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
            rng = 0.05          # 5 cm，至少让 BendyRuler 觉得“有缝”
        # 计算遮挡角度范围
        ang_obs = math.degrees(math.atan2(r, f)) % 360
        width   = max(10.0, math.degrees(math.atan2(r_o, rng)) * 2)
        for i, a in enumerate(angles):
            delta = min(abs(a - ang_obs), 360 - abs(a - ang_obs))
            if delta <= width:
                d_cm = int(rng * 100)
                # 取最近距离
                if dist_cm[i] == 65535 or d_cm < dist_cm[i]:
                    dist_cm[i] = d_cm
                if d_cm < 1:
                    d_cm = 1
    return list(dist_cm)

def send_obstacle(master):
    arr = gen_distance_array(master)   # arr 已是 list
    print('正在发送障碍物信息... OBSTACLE_DISTANCE:', len(arr), 'pts, min=%d max=%d cm' % (min(arr), max(arr)))
    print('[SND] OBSTACLE_DISTANCE %d beams, min=%d cm' % (len(arr), min(arr)))
    master.mav.obstacle_distance_send(
        int(time.time() * 1_000_000),              # 0  time_usec
        mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER, # 1  sensor_type
        arr,                                       # 2  distances[72]
        5,                                         # 3  increment [deg]
        0,                                         # 4  min_distance [cm]
        3000,                                      # 5  max_distance [cm]
        5.0,                                       # 6  increment_f
        0,                                         # 7  angle_offset
        mavutil.mavlink.MAV_FRAME_BODY_FRD,        # 8  frame
    )

def run_auto():
    # 1. 设置参数 - 关键：启用避障并配置算法
    params = {
        'FRAME_CLASS': 2,       # 2=Boat（Rover固件）
        'ARMING_CHECK': 0,      # 跳过自检
        'OA_TYPE': 3,           # 3=BendyRuler+Dijkstra（全局路径规划+实时避障）
        'AVOID_ENABLE': 7,      # 7=启用所有避障功能（关键！）
        'AVOID_MARGIN': 10,     # 安全边距10米（触发避障的阈值）
        'OA_MARGIN_MAX': 20,    # 最大避障边距20米
        'CRUISE_SPEED': 1.5,    # 巡航速度（降低速度给避障留时间）
        'OA_BR_LOOKAHEAD': 20,
    }
    for name, val in params.items():
        param_set(name, float(val))

    # 检查 GPS
    check_gps(gps_check=True)
    
    # 解锁飞控
    vehicle_arming()
    
    # 切换模式为MANUAL
    if not set_mode(0):
        exit()
    # 切换模式为GUIDED
    # if not set_mode(15):
    #     exit()

    """
    # 航点定义 - 障碍物在起点和终点之间
    waypoints = [
        (30.2497209, 120.1523852),  # 起点
        (30.2487078, 120.1527473),  # 终点（障碍物在中间）
    ]
    # 发送航点任务
    send_waypoints(waypoints)
    """
    
    # 告诉飞控航点个数
    num_waypoints = 3
    con.mav.mission_count_send(con.target_system, con.target_component, num_waypoints)
    time.sleep(0.5)

    # 切换模式为AUTO
    if not set_mode(10):
        exit()

    # 稍作等待
    time.sleep(1.0)
    
    # 持续发送障碍物数据（飞控在AUTO模式下会实时使用这些数据）
    while True:
        send_obstacle(con)
        time.sleep(0.1)  # 10Hz发送频率

    # @TODO 等待航行结束，切换为LOITER或HOLD模式

if __name__ == "__main__":
    run_auto()
