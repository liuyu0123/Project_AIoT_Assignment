from pymavlink import mavutil
import time

# 1. 建立连接
con = mavutil.mavlink_connection('COM11', baud=57600)
con.wait_heartbeat()
print("已收到心跳，系统 ID=%d" % con.target_system)



# 2. 发送解锁指令
print("准备解锁...")
con.mav.command_long_send(
    con.target_system,
    con.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,          # confirmation
    1, 0, 0, 0, 0, 0, 0)  # param1=1 表示解锁

# 3. 等 ACK，超时 3 秒
ack = con.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
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

# 4. 再读一次心跳，顺便看 base_mode 的 ARM 标志
msg = con.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
if msg:
    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    print("心跳显示 armed =", armed)