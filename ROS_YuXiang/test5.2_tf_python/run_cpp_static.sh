colcon build --packages-select demo_cpp_tf
source install/setup.bash
ros2 run demo_cpp_tf static_tf_broadcaster