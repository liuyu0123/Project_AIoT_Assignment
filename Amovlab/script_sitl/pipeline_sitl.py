from pymavlink import mavutil
import time, glob, math

con = mavutil.mavlink_connection('udp:10.16.44.241:14551')
con.wait_heartbeat()
print("飞控心跳正常")


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


def run():
    # 检查 GPS
    check_gps(gps_check=True)
    # 解锁飞控
    vehicle_arming()
    # 切换模式
    set_mode(mode_id=15)
    # 运动
    vehicle_trajectory = [
        30.2491416, 120.1524201,
        30.2489285, 120.1520070,
        30.2485485, 120.1522163,
        30.2487524, 120.1527795
    ]
    # running2position(con, speed_ms=3)
    loop_times = 2
    for _ in range(loop_times):
        for i in range(0, len(vehicle_trajectory), 2):
            running2position(con, speed_ms=3, target_lat=vehicle_trajectory[i], target_lon=vehicle_trajectory[i+1])
            time.sleep(10)





if __name__ == "__main__":
    run()

