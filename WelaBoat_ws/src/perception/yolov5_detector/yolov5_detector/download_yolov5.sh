#!/bin/bash

# 获取脚本的绝对路径
SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

# 切换到脚本所在的目录
cd "$SCRIPT_DIR" || exit

echo "当前目录：$(pwd)"

echo "开始下载yolov5"
git clone https://github.com/ultralytics/yolov5.git