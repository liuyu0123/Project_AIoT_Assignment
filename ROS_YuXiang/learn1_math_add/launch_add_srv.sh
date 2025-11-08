colcon build --packages-select add_test_srv
source install/setup.bash
# ros2 launch add_test_srv test.launch.py
ros2 launch add_test_srv test.launch.py weight_factor_a:=2.0 weight_factor_b:=2.0