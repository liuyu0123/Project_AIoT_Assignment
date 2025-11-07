# 生成package，用于实现基于service的加法
ros2 pkg create add_test_srv --build-type ament_python --dependencies rclpy std_msgs interface_test --license Apache-2.0