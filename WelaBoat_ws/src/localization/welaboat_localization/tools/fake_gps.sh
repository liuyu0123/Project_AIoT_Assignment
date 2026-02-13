ros2 topic pub /gps/fix sensor_msgs/msg/NavSatFix "
header:
  frame_id: 'gps_link'
status:
  status: 0
  service: 1
latitude: 30.2741
longitude: 120.1551
altitude: 10.0
position_covariance: [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
position_covariance_type: 2
" -r 1
