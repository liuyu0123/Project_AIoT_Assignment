from pymavlink import mavutil
import time, glob

con = mavutil.mavlink_connection('COM11', baud=57600)
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
    con.mav.command_long_send(
        con.target_system,
        con.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,          # confirmation
        1, 0, 0, 0, 0, 0, 0)  # param1=1 表示解锁

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




def switch_mode(mode_id=4):
    con.mav.command_long_send(
            con.target_system, con.target_component,
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


def vehicle_speed_set(speed=0):
    pass


def run():
    # 检查 GPS
    check_gps(gps_check=False)
    # 解锁飞控
    vehicle_arming()
    # 切换模式
    switch_mode(mode_id=4)
    # 起飞
    vehicle_takeoff(alt=3)
    vehicle_speed_set(speed=1)



if __name__ == "__main__":
    run()
    exit()

    # 5. 打印高度
    while True:
        msg = con.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        alt = msg.relative_alt / 1000
        print("alt=%.1f m" % alt)
        if alt > 2.8:
            break
    print("Reached 3 m → 切换到 STABILIZE 手动降落 或 继续步骤3")