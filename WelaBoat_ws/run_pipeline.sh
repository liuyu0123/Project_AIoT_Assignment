colcon build
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install
conda deactivate
conda deactivate
source install/setup.bash
ros2 launch welaboat_bringup welaboat.launch.py
# ros2 launch welaboat_bringup welaboat_visionOnly.launch.py