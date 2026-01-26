if_launch=$1

colcon build
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install
# source ~/miniconda3/bin/activate lidar_env
# python -m colcon build --packages-select freespace_shoreline --symlink-install
conda deactivate
conda deactivate
source install/setup.bash

if [ $if_launch -eq 1 ]; then
    ros2 launch welaboat_bringup welaboat.launch.py
fi
# ros2 launch welaboat_bringup welaboat_visionOnly.launch.py