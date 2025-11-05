colcon build --packages-select minimal_topic
source install/setup.bash
ros2 run minimal_topic talker
# ros2 run minimal_topic listener