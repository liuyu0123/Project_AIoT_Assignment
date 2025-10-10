colcon build --packages-select fishbot_description
source install/setup.bash

mode_type=3
if [ $mode_type -eq 1 ]; then
ros2 launch fishbot_description display_robot.launch.py
elif [ $mode_type -eq 2 ]; then
ros2 launch fishbot_description display_robot.launch.py mode_path:=$(ros2 pkg prefix fishbot_description)/share/fishbot_description/urdf/first_robot.urdf.xacro
elif [ $mode_type -eq 3 ]; then
ros2 launch fishbot_description display_robot.launch.py mode_path:=$(ros2 pkg prefix fishbot_description)/share/fishbot_description/urdf/fishbot/fishbot.urdf.xacro
fi

echo