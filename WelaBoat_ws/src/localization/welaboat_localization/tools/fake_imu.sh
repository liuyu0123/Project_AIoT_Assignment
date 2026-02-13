#!/bin/bash
CURRENT_TIME=$(date +%s)

ros2 topic pub /imu/data sensor_msgs/msg/Imu "
header:
  stamp:
    sec: $CURRENT_TIME
    nanosec: 0
  frame_id: 'base_link'
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.8
" -r 10