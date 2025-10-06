# colcon build

# colcon build --packages-select demo_cpp_pkg

# colcon build --packages-select demo_python_pkg
# source install/setup.bash
# # ros2 run demo_python_pkg person_node
# # ros2 run demo_python_pkg writer_node
# # ros2 run demo_python_pkg person_node2
# ros2 run demo_python_pkg writer_node2


# colcon build --packages-select demo_cpp_pkg
# source install/setup.bash
# ros2 run demo_cpp_pkg person_node



colcon build --packages-select demo_python_pkg
source install/setup.bash
ros2 run demo_python_pkg learn_thread