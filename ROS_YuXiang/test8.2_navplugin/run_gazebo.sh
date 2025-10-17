set -e
colcon build
source install/setup.bash
# 启动 gazebo
ros2 launch fishbot_description gazebo_sim.launch.py