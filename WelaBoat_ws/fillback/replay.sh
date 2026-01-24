#!/usr/bin/env bash
set -e

# =====================
# 可配置区域
# =====================
INPUT_BAG=$1
OUTPUT_BAG="replay_$(date +%Y%m%d_%H%M%S)"

# 新版本输出 topic（需要录制的）
RECORD_TOPICS=(
  /perception_v2/object
  /perception_v2/debug
)

# 旧版本输出 topic（需要屏蔽的）
EXCLUDE_TOPICS=(
  /perception/object
  /perception/debug
)

# 用于判断 node ready 的 topic（新版本一定会 pub）
READY_TOPIC=/perception_v2/object

# 启动新版本算法
LAUNCH_CMD="ros2 launch perception replay.launch.py"

# =====================
# 执行逻辑
# =====================

if [ -z "$INPUT_BAG" ]; then
  echo "Usage: ./replay.sh <input_bag>"
  exit 1
fi

echo "[1/5] Launch new perception nodes..."
eval "$LAUNCH_CMD &"
LAUNCH_PID=$!

echo "[2/5] Waiting for new node to be ready..."
ros2 topic wait "$READY_TOPIC"

echo "[3/5] Start recording new outputs..."
ros2 bag record "${RECORD_TOPICS[@]}" -o "$OUTPUT_BAG" &
RECORD_PID=$!

sleep 1

echo "[4/5] Play input bag (exclude old outputs)..."
ros2 bag play "$INPUT_BAG" \
  --clock \
  $(printf -- "--exclude-topics %s " "${EXCLUDE_TOPICS[@]}")

echo "[5/5] Playback finished. Cleaning up..."
kill $RECORD_PID
kill $LAUNCH_PID

echo "Replay finished. Output bag: $OUTPUT_BAG"
