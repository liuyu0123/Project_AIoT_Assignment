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



# 录制数据
ros2 bag record -a
# 录制数据，按照--max-bag-duration设定的时间间隔分包（直接删掉无法play）
ros2 bag record -a --max-bag-duration=10

# 播放数据
ros2 bag play rosbag2_2026_01_23-11_28_49/
# 循环播放
ros2 bag play -l rosbag2_2026_01_23-11_28_49/


# 一键启动整个链路
ros2 launch welaboat_bringup welaboat.launch.py
