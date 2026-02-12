colcon build

source ~/miniconda3/bin/activate
conda activate yolov5_env
python -m colcon build --packages-select yolov5_detector --symlink-install
conda deactivate
conda deactivate

source install/setup.bash
# 启动相机驱动节点
ros2 run camera_driver camera_driver
# 启动相机预览节点
ros2 run camera_driver camera_viewer /camera/left/image_raw
ros2 run camera_driver camera_viewer /camera/left/image_rect
ros2 run camera_driver camera_viewer /stereo/disparity_color  #预览视差图
ros2 run camera_driver camera_viewer /yolov5/detections_image  #预览yolov5检测结果
ros2 run camera_driver camera_viewer /fastscnn/segmentation_color  #预览fastscnn语义分割结果
# 启动相机校正节点
ros2 run camera_driver camera_rectified

# 启动双目视差节点
ros2 run stereo_disparity stereo_disparity --ros-args --params-file install/stereo_disparity/share/stereo_disparity/config/sgbm_params.yaml
# 使用launch脚本启动节点
ros2 launch stereo_disparity disparity.launch.py

# 启动单目yolo节点
ros2 run yolov5_detector yolov5_detector_node --ros-args -p \
  model_path:=/home/riba/GitProject/LIUYU/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt

ros2 run yolov5_detector yolov5_detector_node \
  --ros-args \
  -p model_path:=/home/riba/your_model.pt \
  -p input_topic:=/camera/left/image_rect \
  -p output_topic:=/yolov5/detections_image \
  -p conf_thres:=0.5

# 启动conda环境版本
ros2 launch yolov5_detector yolov5.launch.py \
  model_path:=/home/riba/GitProject/AIoT_Gitee/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt
# 指定 Conda 环境和输入话题
ros2 launch yolov5_detector yolov5.launch.py \
  model_path:=/home/riba/GitProject/AIoT_Gitee/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt \
  conda_env:=yolov5_env \
  input_topic:=/camera/left/image_rect

ros2 launch yolov5_detector yolov5_test1.launch.py

ros2 launch yolov5_detector yolov5_test2.launch.py



# 启动fastscnn语义分割节点
ros2 run fastscnn_segmenter fastscnn_segmenter_node \
  --ros-args \
  -p model_path:=/home/riba/Fast-SCNN-pytorch/weights_copy/fast_scnn_citys.pth \
  -p input_topic:=/camera/left/image_rect \
  -p num_classes:=19

ros2 run fastscnn_segmenter fastscnn_segmenter_node \
  --ros-args \
  -p model_path:=/home/riba/Fast-SCNN-pytorch/weights_copy/fast_scnn_water.pth \
  -p input_topic:=/camera/left/image_rect \
  -p num_classes:=2


# 激光雷达USB接口权限
# sudo chmod 666 /dev/ttyUSB0
# 查看点云质量
rviz2 -d src/drivers/unitree_lidar_ros2/rviz/view.rviz 



####################### ROS2 PNG&PCD RECORD [FOR CALIBRATION] ######################
# 先运行run_capture.sh
# 再新开一个终端，运行
ros2 run vision_lidar_capture vision_lidar_capture
# 检查图像是否符合要求
ros2 run camera_driver camera_viewer /camera/left/image_raw



####################### ROS2 BAG RECORD ######################
# 录制数据
ros2 bag record -a
# 录制数据，按照--max-bag-duration设定的时间间隔分包（直接删掉无法play）
ros2 bag record -a --max-bag-duration=10
# 录制数据，mcap格式
ros2 bag record -a --max-bag-duration=10 --storage mcap

# 播放数据
ros2 bag play rosbag2_2026_01_23-11_28_49/
# 循环播放
ros2 bag play -l rosbag2_2026_01_23-11_28_49/


####################### ROS2 BAG PLAY ######################
# 纸板目标
ros2 bag play -l /home/riba/GitProject/LIUYU/WelaBoat_ws/record/TargetBoardData/rosbag2_2026_01_24-15_27_37/
# 3D目标检测
ros2 bag play -l /home/riba/GitProject/LIUYU/WelaBoat_ws/record/TargetBoardData/rosbag2_2026_01_24-15_27_37/rosbag2_2026_01_24-15_27_37_0.mcap


# 获取bag信息
ros2 bag info /home/riba/GitProject/LIUYU/WelaBoat_ws/record/Target3DNew/rosbag2_2026_01_24-16_31_52_fillback_20260124_222024/rosbag2_2026_01_24-16_31_52_fillback_20260124_222024_0.db3

####################### ROS2 BAG FILLBACK ######################
# ros2 launch welaboat_bringup fillback.launch.py
cd ws
bash fillback.sh /home/riba/GitProject/LIUYU/WelaBoat_ws/record/Target3DNew/rosbag2_2026_01_24-16_31_52/rosbag2_2026_01_24-16_31_52_1.mcap




####################### ROS2 DEBUG ######################
# 1. colcon build所有节点，只要有修改，都需要重新编译。（可以选择只编译修改的节点）
# 注意：需要调试的节点，必须使用 --cmake-args -DCMAKE_BUILD_TYPE=Debug 进行编译，否则无法调试
colcon build --packages-select lidar_vision_fusion --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 2. 启动节点
source install/setup.bash
ros2 launch welaboat_bringup welaboat_debug.launch.py

# 3. 播放bag，建议从指定时刻开始播放
# 通过rviz或者foxglove查看数据，记住时间戳。
--start-offset # 跳过多少秒（相对起始时刻的偏移量，不是绝对时间戳）
--duration # 持续多少秒
ros2 bag play your_file.db3 --start-offset 120
ros2 bag play your_file.db3 --start-offset 120 --duration 5
ros2 bag play /home/riba/GitProject/LIUYU/WelaBoat_ws/record/Target3DNew/rosbag2_2026_01_24-16_31_52/rosbag2_2026_01_24-16_31_52_0.mcap \
  --clock --start-offset 6.56862448

# 左边问题时间通过foxglove查看，右边通过ros2 bag info查看  
1769243519.465698421 - 1769243512.897073941 = 6.56862448
ros2 bag info /home/riba/GitProject/LIUYU/WelaBoat_ws/record/Target3DNew/rosbag2_2026_01_24-16_31_52/rosbag2_2026_01_24-16_31_52_0.mcap

# 4. 进入vscode，打断点调试



####################### ROS2 导航 ######################
# 启动 pointcloud_to_laserscan 点云转换为激光线
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
  --ros-args \
  -r cloud_in:=/unilidar/cloud \
  -r scan:=/scan \
  -p target_frame:=base_link \
  -p min_height:=-0.3 \
  -p max_height:=0.3


####################### ROS2 导航 NAVIGATION [方案1]（失败） ######################
# 启动 costmap 
ros2 run nav2_costmap_2d costmap_server \
  --ros-args \
  -p use_sim_time:=false \
  --params-file src/navigation/config/nav2_costmap_only.yaml

ros2 run nav2_costmap_2d costmap_server \
  --ros-args \
  -r __node:=local_costmap \
  -r __ns:=/local_costmap \
  --params-file src/navigation/config/costmap_server.yaml


ros2 run nav2_costmap_2d nav2_costmap_2d \
  --ros-args \
  -p use_sim_time:=false \
  --params-file src/navigation/config/nav2_costmap_only.yaml

ros2 run nav2_costmap_2d nav2_costmap_2d \
  --ros-args \
  --params-file src/navigation/config/costmap.yaml

# 通过容器组件的方式启动costmap_server
# step1 新终端运行
# ros2 run rclcpp_components component_container
ros2 run rclcpp_components component_container --ros-args -r __node:=ComponentManager

# step2 新终端运行
ros2 component load \
  /ComponentManager \
  nav2_costmap_2d \
  nav2_costmap_2d::CostmapServer \
  --node-name local_costmap \
  --node-namespace /local_costmap \
  --param-file src/navigation/config/costmap_server_component.yaml

# 播包
ros2 bag play -l rosbag2_2026_02_03-12_10_22_0.mcap --clock
# 启动costmap_server
colcon build --packages-select welaboat_bringup
ros2 launch welaboat_bringup costmap_container.launch.py


####################### ROS2 导航 NAVIGATION [方案2] ######################
# 方案2：nav2_bringup
# rm -rf build install log
colcon build --symlink-install
colcon build --packages-select welaboat_bringup --symlink-install
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  params_file:=src/welaboat/welaboat_bringup/welaboat_bringup/config/nav2_minimal.yaml \
  autostart:=true

# 方案3
ros2 launch welaboat_bringup nav2_odom_only.launch.py
ros2 launch welaboat_bringup nav2_odom_only.launch.py use_sim_time:=true


############################ 带有反馈的导航仿真 #####################
#0. 编译 welaboat_bringup 即 nav 节点
colcon build --packages-select welaboat_bringup --symlink-install
source install/setup.bash
#1. 启动 fake_odom_control
ros2 run simu_localization fake_odom_control
ros2 run simu_localization fake_odom_control use_sim_time:=true
#2. 启动 nav2_bringup
ros2 launch welaboat_bringup nav2_odom_only.launch.py
ros2 launch welaboat_bringup nav2_odom_only.launch.py use_sim_time:=true
#3. 启动 rviz
rviz2 -d rviz/simulation.rviz


############################ 带有反馈的导航仿真，含map的完整流程 #####################
# 1. 新终端启动map
# 1.1 启动map_server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/riba/GitProject/LIUYU/WelaBoat_ws/src/welaboat/welaboat_bringup/welaboat_bringup/map/map.yaml
# 1.2 新终端运行configure和activate
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
# 1.3 验证map
ros2 topic echo /map

# 2. 启动odom
ros2 run simu_localization fake_odom_control

# 3. 新终端启动tf
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

# 4. 新终端启动nav2
ros2 launch welaboat_bringup nav2_odom_only.launch.py

# 5. 新终端启动rviz2
rviz2 -d rviz/simulation.rviz


# navigation 导航仿真一键启动
ros2 launch welaboat_bringup welaboat_navigation.launch.py




####################### ROS2 LAUNCH ######################
# 一键启动整个链路
ros2 launch welaboat_bringup welaboat.launch.py
