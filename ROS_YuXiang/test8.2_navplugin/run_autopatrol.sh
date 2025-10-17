source install/setup.bash
# ros2 launch autopatrol_robot autopatrol.launch.py > autopatrol.log 2>&1 
ros2 launch autopatrol_robot autopatrol.launch.py > "autopatrol_$(date +%Y%m%d_%H%M%S).log" 2>&1