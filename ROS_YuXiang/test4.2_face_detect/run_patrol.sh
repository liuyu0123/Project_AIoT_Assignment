colcon build
source install/setup.bash
ros2 interface show chapt4_interfaces/srv/Patrol
ros2 run demo_cpp_service turtle_control
# ros2 run turtlesim turtlesim_node  # run this after turtle_control in another terminal
# remember: always need to source in new terminal.