colcon build
source install/setup.bash
# ros2 launch fishbot_description display_robot.launch.py
# ros2 launch fishbot_description display_robot.launch.py model:=.../first_robot.urdf.xacro
ros2 launch fishbot_description display_robot.launch.py model:=....../fishbot_description/urdf/first_robot.urdf.xacro
