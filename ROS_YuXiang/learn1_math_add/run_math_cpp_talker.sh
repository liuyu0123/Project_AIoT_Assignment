colcon build --packages-select math_cpp
source install/setup.bash

ros2 run math_cpp talker
# ros2 topic echo hello