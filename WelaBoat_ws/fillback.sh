#!/usr/bin/env bash
set -e


# =====================
# 可配置区域
# =====================
INPUT_BAG=$1
TS=$(date +%Y%m%d_%H%M%S)               # 只算一次
OUTPUT_BAG="${INPUT_BAG%.*}_fillback_$TS"   # 去掉扩展名再拼
MERGED_BAG="${INPUT_BAG%.*}_merged_$TS"
echo "[input] input bag: $INPUT_BAG"
echo "[output] output bag: $OUTPUT_BAG"
echo "[output] merged bag: $MERGED_BAG"


# 新版本输出 topic（需要录制的）
RECORD_TOPICS=(
  /fused/objects_marker_debug
)

# 旧版本输出 topic（需要屏蔽的）
EXCLUDE_TOPICS=(
  # /fused/objects_marker
)

# 用于判断 node ready 的 topic（新版本一定会 pub）
READY_TOPIC=/fused/objects

# 启动新版本算法
LAUNCH_CMD="ros2 launch welaboat_bringup fillback.launch.py"

# =====================
# 执行逻辑
# =====================
# 构建项目
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
conda deactivate
conda deactivate
source install/setup.bash


# 检查BAG是否存在
if [ -z "$INPUT_BAG" ]; then
  echo "Usage: ./fillback.sh <input_bag>"
  exit 1
fi

echo "[1/6] Launch new perception nodes..."
eval "$LAUNCH_CMD &"
LAUNCH_PID=$!

echo "[2/6] Waiting for new node to be ready..."
# ==== 新增函数：轮询等待 topic ====
wait_for_topic() {
  local topic=$1
  local timeout=${2:-30}   # 默认 30 秒
  local t0=$(date +%s)
  echo "等待 topic <$topic> 出现（超时 $timeout 秒）..."
  while ! ros2 topic list | grep -Fx "$topic" >/dev/null 2>&1; do
    sleep 0.5
    local now=$(date +%s)
    if (( now - t0 >= timeout )); then
      echo "超时仍未检测到 <$topic>，退出"
      exit 1
    fi
  done
  echo "topic <$topic> 已就绪"
}

# ros2 topic wait "$READY_TOPIC" --timeout 30
wait_for_topic "$READY_TOPIC" 30


echo "[3/6] Start recording new outputs..."
ros2 bag record "${RECORD_TOPICS[@]}" -o "$OUTPUT_BAG"  --storage mcap &
RECORD_PID=$!

sleep 1

echo "[4/6] Play input bag (exclude old outputs)..."
ros2 bag play "$INPUT_BAG" \
  --clock \
  $(printf -- "--exclude-topics %s " "${EXCLUDE_TOPICS[@]}")

echo "[5/6] Fillback finished. Cleaning up..."
# kill $RECORD_PID
# kill $LAUNCH_PID

# # 录包进程
# kill $RECORD_PID 2>/dev/null || true
# # 把 launch 及其所有子节点一起杀
# pkill -P $LAUNCH_PID 2>/dev/null || true
# kill $LAUNCH_PID 2>/dev/null || true

# 停止录制
kill -INT $RECORD_PID 2>/dev/null
wait $RECORD_PID 2>/dev/null || true
# 杀 launch 主进程及其所有子节点
kill -INT -$LAUNCH_PID 2>/dev/null || true   # 负号表示进程组
wait $LAUNCH_PID 2>/dev/null || true


echo "Replay finished. Output bag: $OUTPUT_BAG"


echo "[6/6] Merge new outputs with input bag..."
wait $LAUNCH_PID || true
python3 fillback/merge_bag.py \
  --base ${INPUT_BAG} \
  --replay ${OUTPUT_BAG} \
  --out ${MERGED_BAG} \
  --override-topics ${RECORD_TOPICS}

echo "[done] merged bag ready: ${MERGED_BAG}"