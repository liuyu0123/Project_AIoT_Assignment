colcon build --packages-select interface_test
colcon build --packages-select add_test_srv
source install/setup.bash
ros2 run add_test_srv talker
# ros2 run add_test_srv listener
# ros2 run add_test_srv listener 2 4

# 动态修改参数（无需重启）
# ros2 param set /add_server weight_a 1.0
# ros2 param set /add_server weight_b 1.0