# 编译整个工作空间
colcon build

# 如果遇到权限问题，可以添加 --symlink-install
# colcon build --packages-select robot_hardware --symlink-install

source install/setup.bash

launch_type=1

if [ $launch_type -eq 1 ]; then
  # 启动整个系统
  ros2 launch robot_hardware hardware_system.launch.py
else
  # 单独运行各节点
  ros2 run robot_hardware button_publisher
  ros2 run robot_hardware led_controller --ros-args -p gpio_pin:=71
  ros2 run robot_hardware logic_controller
  ros2 run robot_hardware servo_controller --ros-args -p gpio
fi


# # 点亮 LED
# ros2 topic pub --once /led_control std_msgs/msg/Bool "{data: true}"
# # 熄灭 LED
# ros2 topic pub --once /led_control std_msgs/msg/Bool "{data: false}"

# # 舵机转到最大位置
# ros2 topic pub --once /servo_control std_msgs/msg/Float32 "{data: 1.0}"
# # 舵机转到中间位置
# ros2 topic pub --once /servo_control std_msgs/msg/Float32 "{data: 0.0}"
# # 舵机转到最小位置
# ros2 topic pub --once /servo_control std_msgs/msg/Float32 "{data: -1.0}"

