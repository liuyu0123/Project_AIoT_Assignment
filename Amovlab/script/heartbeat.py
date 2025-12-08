from pymavlink import mavutil
import time
import glob

# 1. 找到飞控串口（glob 自动展开通配符）
try:
    port = glob.glob('/dev/serial/by-id/usb-ArduPilot_fmuv3_*-if00')[0]  # 取第一个匹配
    print("检测到串口设备:",port)
except IndexError:
    raise SystemExit('找不到飞控串口，确认是否已插上')

# 1. 连接（115200 是 ArduPilot 默认）
# GPIO串口
# con = mavutil.mavlink_connection('/dev/serial0', baud=115200)
# USB串口
# con = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
# USB串口：指定设备更稳定，不会随插拔顺序变化
# con = mavutil.mavlink_connection('/dev/serial/by-id/usb-Generic_fmuv3_*-if00', baud=115200)
con = mavutil.mavlink_connection(port, baud=115200)

# 2. 等待第一帧心跳
print("Waiting heartbeat...")
con.wait_heartbeat()
print("Got heartbeat! sysid=%u compid=%u" % (con.target_system, con.target_component))

# 3. 持续打印心跳（1 Hz）
while True:
    msg = con.recv_match(type='HEARTBEAT', blocking=True)
    print("mode=%s armed=%s" % (mavutil.mode_string_v10(msg), msg.base_mode & 0x80))
    time.sleep(1)