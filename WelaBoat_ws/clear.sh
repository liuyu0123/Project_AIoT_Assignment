ps aux | grep fillback.launch.py
pkill -f fillback.launch.py

# 杀掉所有由 launch 启动的节点
pkill -f "launch.*fillback.launch.py"
# 再补一刀
pkill -f "stereo_disparity_node"
pkill -f "yolov5_detector_node"
pkill -f "fastscnn_segmenter_node"
pkill -f "lidar_vision_fusion_node"
pkill -f "multi_lidar_merge_node"


ros2 daemon stop && sleep 1 && ros2 daemon start