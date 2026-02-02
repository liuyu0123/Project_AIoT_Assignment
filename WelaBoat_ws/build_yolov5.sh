colcon build
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install
conda deactivate