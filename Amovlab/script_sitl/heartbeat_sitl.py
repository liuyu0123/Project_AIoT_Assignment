from pymavlink import mavutil
import time


# 1. 连接USB串口（115200 是 ArduPilot 默认）
# 直连飞控串口
# con = mavutil.mavlink_connection('COM7', baud=115200)
# 通过数传模块连接
# con = mavutil.mavlink_connection('COM11', baud=57600)
# 连接 SITL 转发出来的 UDP
con = mavutil.mavlink_connection('udp:10.16.44.241:14550')


# 2. 等待第一帧心跳
print("Waiting heartbeat...")
con.wait_heartbeat()
print("Got heartbeat! sysid=%u compid=%u" % (con.target_system, con.target_component))

# 3. 持续打印心跳（1 Hz）
while True:
    msg = con.recv_match(type='HEARTBEAT', blocking=True)
    print("mode=%s armed=%s" % (mavutil.mode_string_v10(msg), msg.base_mode & 0x80))
    time.sleep(1)