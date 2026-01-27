#!/usr/bin/env bash
# 除了本脚本外，需要手动安装的环境：
# sudo apt install -y ros-$ROS_DISTRO-vision-msgs
# sudo apt install -y python3-colcon-common-extensions
# 安装华腾威视工业相机驱动
# cd ~
# git clone https://github.com/liuyu0123/huateng_vision_sdk.git
# cd huateng_vision_sdk/linuxSDK_V2.1.0.49
# sudo bash install.sh



# WelaBoat 项目根目录
repo_root=$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && \
            while [[ ! -d src ]] && [[ $PWD != / ]]; do cd ..; done; pwd)
[[ -d $repo_root/src ]] || { echo "找不到仓库根目录"; exit 1; }
echo "WelaBoat项目根路径为： ${repo_root}"


# 回到 Welaoat 根目录
cd $repo_root

# 删除 package
dirs=(
    src/develop_tools/vision_lidar_capture
    src/drivers/camera_driver
    src/drivers/unitree_lidar_ros2
    src/drivers/unitree_lidar_sdk
    src/perception/fastscnn_segmenter
    src/perception/freespace_shoreline
    src/perception/lidar_vision_fusion
    src/perception/stereo_disparity
    src/perception/yolov5_detector
    src/welaboat/welaboat_bringup
)
for d in "${dirs[@]}"; do
    if [[ -e "$d" ]]; then
        echo "准备删除：$d"
        rm -rf "$d"
    else
        echo "路径不存在：$d"
    fi
done
cd src/develop_tools; bash create_pkg_tools.sh; cd $repo_root
cd src/drivers; bash create_pkg_drivers.sh; cd $repo_root
cd src/perception; bash create_pkg_percept.sh; cd $repo_root
cd src/welaboat; bash create_pkg_system.sh; cd $repo_root
git restore .


# 拉取 unitree lidar 官方SDK
cd src/drivers
rm -rf unilidar_sdk
rm -rf unitree_lidar_sdk unitree_lidar_ros2
echo "开始拉取宇树激光雷达SDK： unilidar_sdk"
git clone https://github.com/unitreerobotics/unilidar_sdk.git
mv unilidar_sdk/unitree_lidar_sdk .
mv unilidar_sdk/unitree_lidar_ros2/src/unitree_lidar_ros2 .
rm -rf unilidar_sdk
git restore .

# 拉取 yolov5 和 fastscnn 模型
if [ -e "$HOME/yolov5" ]; then
    echo "yolov5已经存在，跳过部署..."
else
    echo "yolov5不存在，开始部署..."
    cd ~
    git clone https://github.com/ultralytics/yolov5.git
fi

if [ -e "$HOME/Fast-SCNN-pytorch" ]; then
    echo "Fast-SCNN-pytorch已经存在，跳过部署..."
else
    echo "Fast-SCNN-pytorch不存在，开始部署..."
    cd ~
    # git clone https://github.com/Tramac/Fast-SCNN-pytorch.git
    git clone https://github.com/liuyu0123/Fast-SCNN-pytorch.git -b develop
fi

# 配置 yolov5 检测模型
yolov5_model_path="$HOME/huateng_vision_sdk/model_yolov5/yolov5s.pt"
if [ -e $yolov5_model_path ]; then
    echo "发现 yolov5 模型文件，拷贝到 workspace 中..."
    cd $repo_root
    mkdir -p src/perception/yolov5_detector/model
    cp $yolov5_model_path src/perception/yolov5_detector/model
else
    echo "未发现 yolov5 模型文件，请手动配置..."
fi

# 构建工程
cd $repo_root
echo "开始构建项目 WelaBoat..."
colcon build