# 纯激光freespace节点
ros2 pkg create --build-type ament_cmake freespace_lidar_node \
  --dependencies rclcpp sensor_msgs nav_msgs pcl_ros libpcl-all-dev opencv2

