# 双目视差
ros2 pkg create stereo_disparity --build-type ament_python --dependencies rclpy --license Apache-2.0

# yolov5目标检测
ros2 pkg create yolov5_detector \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge std_msgs vision_msgs \
  --license Apache-2.0

# fastscnn语义分割
ros2 pkg create fastscnn_segmenter \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge std_msgs vision_msgs \
  --license Apache-2.0

# 激光雷达
# lidar_processor
# 直接用宇树的官方SDK的ROS2版本。

# 视觉融合
vision_fusion

# 多源感知融合
# radar_vision_fusion
ros2 pkg create lidar_vision_fusion \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs cv_bridge std_msgs vision_msgs message_filters \
  --license Apache-2.0

