source install/setup.bash

ros2 service call /patrol chapt4_interfaces/srv/Patrol "{target_x: 9.0, target_y: 2.0}"