# yolov5目标检测节点必须在conda环境中单独构建，该conda环境必须和系统python版本一致（例如ubuntu22.04必须为python3.10）
source ~/miniconda3/bin/activate
conda activate yolov5_env
python -m colcon build --packages-select yolov5_detector --symlink-install

# 构建完成之后，在任何终端下执行都可以。
# conda deactivate
# conda deactivate

# 启动yolov5目标检测节点
yolov5_model="/home/riba/GitProject/LIUYU/WelaBoat_ws/src/perception/yolov5_detector/model/yolov5s.pt"
ros2 run yolov5_detector yolov5_detector_node --ros-args -p  model_path:=$yolov5_model

