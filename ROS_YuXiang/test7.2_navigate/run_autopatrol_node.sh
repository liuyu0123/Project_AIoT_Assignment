source install/setup.bash
ros2 run autopatrol_robot patrol_node --ros-args --params-file install/autopatrol_robot/share/autopatrol_robot/config/patrol_config.yaml \
> "patrol_node_$(date +%Y%m%d_%H%M%S).log" 2>&1