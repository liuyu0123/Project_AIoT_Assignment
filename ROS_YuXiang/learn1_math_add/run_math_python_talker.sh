colcon build --packages-select math_python
source install/setup.bash

ros2 run math_python talker
# ros2 topic echo hello