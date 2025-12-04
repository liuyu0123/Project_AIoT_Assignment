# !/usr/bin/env python3
# -*- coding: utf-8 -*-

print("Start simulator (SITL)")
import dronekit_sitl

# 启动默认版本的 SITL
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

# 导入 DroneKit
from dronekit import connect, VehicleMode

print("Connecting to vehicle on: %s" % connection_string)
# 连接飞行器
vehicle = connect(connection_string, wait_ready=True)

# 读取并打印一些状态
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)   # 可设置

# 关闭连接
vehicle.close()

# 关闭仿真器
sitl.stop()
print("Completed")