# 只采集camera_raw和lidar点云数据，保存为png和pcd文件。
colcon build
source ~/miniconda3/bin/activate lidar_env
python -m colcon build --packages-select vision_lidar_capture --symlink-install
# python -m colcon build --packages-select vision_lidar_capture
conda deactivate
conda deactivate
source install/setup.bash
# 给激光雷达添加读写权限
# sudo chmod 666 /dev/ttyUSB0
# 查看激光雷达数据
# rviz2 -d src/drivers/unitree_lidar_ros2/rviz/view.rviz 

# 启动节点（采集数据，用来标定）
ros2 launch welaboat_bringup capture.launch.py

# 另开一个终端，运行
# ros2 run vision_lidar_capture vision_lidar_capture