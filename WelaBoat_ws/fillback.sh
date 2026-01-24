#!/usr/bin/env bash
set -e

# =====================
# 可配置区域
# =====================
INPUT_BAG=$1
TS=$(date +%Y%m%d_%H%M%S)
OUTPUT_BAG="${INPUT_BAG%.*}_fillback_$TS"
MERGED_BAG="${INPUT_BAG%.*}_merged_$TS"

echo "[input] input bag:  $INPUT_BAG"
echo "[output] output bag: $OUTPUT_BAG"
echo "[output] merged bag: $MERGED_BAG"

# 新版本输出 topic（需要录制的）
RECORD_TOPICS=(
  /debug/fused/objects_markers
)

# 旧版本输出 topic（需要屏蔽的）
EXCLUDE_TOPICS=(
  # /fused/objects_markers
)

# 新版本 ready topic（注意 namespace）
READY_TOPIC=/debug/fused/objects

# 启动新版本算法
LAUNCH_CMD="ros2 launch welaboat_bringup fillback.launch.py"

# =====================
# 工程构建
# =====================
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug

source ~/miniconda3/bin/activate yolov5_env
colcon build --packages-select yolov5_detector fastscnn_segmenter \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
conda deactivate

source install/setup.bash

# =====================
# 参数检查
# =====================
if [ -z "$INPUT_BAG" ]; then
  echo "Usage: ./fillback.sh <input_bag>"
  exit 1
fi

# =====================
# 工具函数
# =====================
wait_for_topic() {
  local topic=$1
  local timeout=${2:-30}
  local t0=$(date +%s)

  echo "等待 topic <$topic>（超时 $timeout 秒）..."
  while ! ros2 topic list | grep -Fx "$topic" >/dev/null 2>&1; do
    sleep 0.5
    local now=$(date +%s)
    if (( now - t0 >= timeout )); then
      echo "❌ 超时：未检测到 <$topic>"
      exit 1
    fi
  done
  echo "✅ topic <$topic> 已就绪"
}

# =====================
# [1/7] 启动 rosbag play（先占 ROS graph，但暂停）
# =====================
echo "[1/7] Start rosbag play (paused)..."
ros2 bag play "$INPUT_BAG" \
  --clock \
  --pause \
  $(printf -- "--exclude-topics %s " "${EXCLUDE_TOPICS[@]}") &
PLAY_PID=$!

sleep 2   # 给 rosbag2 时间 advertise

# =====================
# [2/7] 启动新版本算法
# =====================
echo "[2/7] Launch new perception nodes..."
eval "$LAUNCH_CMD &"
LAUNCH_PID=$!

# =====================
# [3/7] 等待算法 ready
# =====================
wait_for_topic "$READY_TOPIC" 30

# =====================
# [4/7] 启动录制（只录 debug）
# =====================
echo "[4/7] Start recording new outputs..."
ros2 bag record "${RECORD_TOPICS[@]}" \
  -o "$OUTPUT_BAG" \
  --storage mcap &
RECORD_PID=$!

sleep 1

# =====================
# [5/7] 解除 rosbag play pause，正式回灌
# =====================
echo "[5/7] Resume rosbag play..."
ros2 service call /rosbag2_player/play std_srvs/srv/Trigger

# =====================
# [6/7] 等待 rosbag play 结束
# =====================
wait $PLAY_PID || true

echo "[6/7] Fillback finished. Cleaning up..."

# 停止录制
kill -INT $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true

# 杀掉 launch（进程组）
kill -INT -$LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

echo "Replay finished. Output bag: $OUTPUT_BAG"

# =====================
# [7/7] 合并 bag
# =====================
echo "[7/7] Merge new outputs with input bag..."
python3 fillback/merge_bag.py \
  --base "$INPUT_BAG" \
  --replay "$OUTPUT_BAG" \
  --out "$MERGED_BAG" \
  --override-topics "${RECORD_TOPICS[@]}"

echo "✅ merged bag ready: $MERGED_BAG"
