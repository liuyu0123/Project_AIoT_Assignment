#1 create map

#2 map_server
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=/path/to/map.yaml
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/riba/GitProject/LIUYU/WelaBoat_ws/src/welaboat/welaboat_bringup/welaboat_bringup/map/map.yaml

#3 configure / activate
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate

ros2 topic echo /map --once

#fix1 map-odom transform
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

#4 global_frame: map [in nav2_params.yaml]

#5 fake_odom_control, nav2_bringup, rviz

#6 check goal in rviz

