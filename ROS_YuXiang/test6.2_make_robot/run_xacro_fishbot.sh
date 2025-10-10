colcon build --packages-select fishbot_description
source install/setup.bash
# ros2 launch fishbot_description display_robot.launch.py
# ros2 launch fishbot_description display_robot.launch.py model:=.../fishbot_description/urdf/fishbot.urdf.xacro
# ros2 launch fishbot_description display_robot.launch.py model:=.../fishbot_description/urdf/fishbot/fishbot2.urdf.xacro
ros2 launch fishbot_description display_robot.launch.py model:=$(ros2 pkg prefix fishbot_description)/share/fishbot_description/urdf/fishbot/fishbot2.urdf.xacro