#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
cd "$SCRIPT_DIR" || exit 1   # 切换失败就退出

# 下面写你真正的业务逻辑
echo "当前已位于脚本所在目录：$(pwd)"

colcon build
source install/setup.bash
# 启动 micro_ros_agent 节点，传输协议为 UDP4，端口为 8888
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888