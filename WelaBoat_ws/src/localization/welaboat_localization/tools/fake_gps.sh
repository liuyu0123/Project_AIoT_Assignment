#!/bin/bash
# 使用当前时间戳
CURRENT_TIME=$(date +%s)

ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "
header:
  stamp:
    sec: $CURRENT_TIME
    nanosec: 0
  frame_id: 'gps_link'
status:
  status: 0
  service: 1
latitude: 30.2742
longitude: 120.1551
altitude: 10.0
position_covariance: [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
position_covariance_type: 2
" -r 1