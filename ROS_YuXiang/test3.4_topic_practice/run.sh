colcon build --packages-select status_interfaces status_publisher status_display
source install/setup.bash
# ros2 run status_interfaces turtle_control
# ros2 interface show status_interfaces/msg/SystemStatus
ros2 run status_publisher sys_status_pub
# ros2 run status_display hello_qt
ros2 run status_display sys_status_display