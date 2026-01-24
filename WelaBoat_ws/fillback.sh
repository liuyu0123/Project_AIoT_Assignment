#!/usr/bin/env bash
set -e

if [ $# -lt 1 ]; then
  echo "Usage: $0 <rosbag_dir_or_mcap>"
  exit 1
fi

INPUT_PATH=$(realpath "$1")


colcon build
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install
conda deactivate
conda deactivate
source install/setup.bash


########################################
# 1. 统一解析成 rosbag 目录
########################################
if [ -f "$INPUT_PATH" ]; then
  # 传的是 .mcap / .db3
  BAG_DIR=$(dirname "$INPUT_PATH")
elif [ -d "$INPUT_PATH" ]; then
  # 直接传的是 bag 目录
  BAG_DIR="$INPUT_PATH"
else
  echo "[ERROR] Invalid input: $INPUT_PATH"
  exit 1
fi

# sanity check
if [ ! -f "$BAG_DIR/metadata.yaml" ]; then
  echo "[ERROR] $BAG_DIR is not a valid rosbag2 directory"
  exit 1
fi

echo "[fillback] base bag dir: $BAG_DIR"

########################################
# 2. 准备输出目录
########################################
TS=$(date +%Y%m%d_%H%M%S)
FILLBACK_BAG="${BAG_DIR}_fillback_${TS}"
MERGED_BAG="${BAG_DIR}_merged_${TS}"

########################################
# 3. 启动 fillback 节点
########################################
echo "[1/6] Launch fillback nodes (use_sim_time)..."
ros2 launch welaboat_bringup fillback.launch.py use_sim_time:=true &
LAUNCH_PID=$!
sleep 3

########################################
# 4. 录制 fillback 结果
########################################
echo "[2/6] Start recording fillback topics..."
ros2 bag record \
  /debug/fused/objects_markers \
  --use-sim-time \
  -o "$FILLBACK_BAG" \
  --storage mcap &
REC_PID=$!
sleep 2

########################################
# 5. 回放原始 bag
########################################
echo "[3/6] Play input bag with clock (slow rate)..."
ros2 bag play "$BAG_DIR" \
  --clock \
  --rate 0.3

########################################
# 6. 停止录制 & 节点
########################################
echo "[4/6] Stop recording..."
kill $REC_PID
wait $REC_PID || true

echo "[5/6] Stop fillback nodes..."
kill $LAUNCH_PID
wait $LAUNCH_PID || true

########################################
# 7. 合并 bag
########################################
echo "[6/6] Merge bags..."
python3 fillback/merge_bag.py \
  --base   "$BAG_DIR" \
  --replay "$FILLBACK_BAG" \
  --out    "$MERGED_BAG" \
  --override /debug/fused/objects_markers

echo "[DONE] merged bag -> $MERGED_BAG"
