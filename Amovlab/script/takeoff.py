from pymavlink import mavutil
import time

# con = mavutil.mavlink_connection('/dev/serial0', baud=115200)
con = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
con.wait_heartbeat()
print("Heartbeat OK")

# 1. 解锁前检查
while True:
    msg = con.recv_match(type='SYS_STATUS', blocking=True)
    if msg.onboard_control_sensors_health & 0x04:  # GPS OK
        break
    print("Waiting GPS 3D fix...")
    time.sleep(1)

# 2. 切 GUIDED
con.mav.set_mode_send(
        con.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4)  # 4=GUIDED
print("Mode set to GUIDED")

# 3. 解锁
con.mav.command_long_send(
    con.target_system, con.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
print("Arming...")
time.sleep(2)

# 4. 起飞 3 m
con.mav.command_long_send(
    con.target_system, con.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 3)  # param7=alt
print("Takeoff 3 m sent")

# 5. 打印高度
while True:
    msg = con.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    alt = msg.relative_alt / 1000
    print("alt=%.1f m" % alt)
    if alt > 2.8:
        break
print("Reached 3 m → 切换到 STABILIZE 手动降落 或 继续步骤3")