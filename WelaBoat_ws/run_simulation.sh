colcon build --packages-select welaboat_bringup
source install/setup.bash
ros2 launch welaboat_bringup welaboat_simulation.launch.py