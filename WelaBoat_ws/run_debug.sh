rm -rf install build log
colcon build --symlink-install
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install
conda deactivate
conda deactivate
colcon build --packages-select lidar_vision_fusion --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install/setup.bash
# ros2 launch welaboat_bringup welaboat_debug.launch.py