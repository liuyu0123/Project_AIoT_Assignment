# 1. new terminal
colcon build
source install/setup.bash
ros2 run demo_cpp_service turtle_control


# 2. new terminal
source install/setup.bash
ros2 run turtlesim turtlesim_node

# 3. new terminal
source install/setup.bash
ros2 run demo_cpp_service patrol_client

