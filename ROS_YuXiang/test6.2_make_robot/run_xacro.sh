colcon build
source install/setup.bash
# ros2 launch fishbot_description display_robot.launch.py
ros2 launch fishbot_description display_robot.launch.py model:=.../fishbot_description/urdf/fishbot.urdf.xacro
