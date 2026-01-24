#!/usr/bin/env bash
set -e

# =====================
# 参数
# =====================
INPUT_BAG=$1
if [ -z "$INPUT_BAG" ]; then
  echo "Usage: ./fillback.sh <input_bag>"
  exit 1
fi

TS=$(date +%Y%m%d_%H%M%S)
OUTPUT_BAG="${INPUT_BAG%.*}_fillback_$TS"
MERGED_BAG="${INPUT_BAG%.*}_merged_$TS"

echo "[input ] $INPUT_BAG"
echo "[record] $OUTPUT_BAG"
echo "[merge ] $MERGED_BAG"

# =====================
# Topic 配置
# =====================
RECORD_TOPICS=(
  /fused/objects_marker_debug
)

EXCLUDE_TOPICS=(
  # /fused/objects_marker
)

READY_TOPIC=/fused/objects

LAUNCH_CMD="ros2 launch welaboat_bringup fillback.launch.py use_sim_time:=true"

# =====================
# Build & env
# =====================
echo "[build] colcon build..."
colcon build --symlink-install
source install/setup.bash

# =====================
# 启动新算法
# =====================
echo "[1/7] Launch fillback nodes..."
eval "$LAUNCH_CMD &"
LAUNCH_PID=$!
sleep 2

# =====================
# 等 topic 出现
# =====================
wait_for_topic() {
  local topic=$1
  local timeout=${2:-30}
  local t0=$(date +%s)

  echo "Waiting for topic <$topic> ..."
  while ! ros2 topic list | grep -Fx "$topic" >/dev/null; do
    sleep 0.5
    if (( $(date +%s) - t0 > timeout )); then
      echo "Timeout waiting for $topic"
      exit 1
    fi
  done
  echo "Topic <$topic> ready"
}

wait_for_topic "$READY_TOPIC" 30

# =====================
# 等 subscriber 真正连上
# =====================
echo "[2/7] Waiting for subscribers..."
sleep 3

# =====================
# 开始录制
# =====================
echo "[3/7] Start recording..."
ros2 bag record \
  "${RECORD_TOPICS[@]}" \
  -o "$OUTPUT_BAG" \
  --storage mcap &
RECORD_PID=$!
sleep 2

# =====================
# 播放 bag（关键）
# =====================
echo "[4/7] Play input bag..."
ros2 bag play "$INPUT_BAG" \
  --clock \
  --wait-for-all-acked \
  --qos-profile-overrides-path fillback/qos_override.yaml \
  $(printf -- "--exclude-topics %s " "${EXCLUDE_TOPICS[@]}")

# =====================
# 清理
# =====================
echo "[5/7] Cleanup..."
kill -INT $RECORD_PID 2>/dev/null || true
wait $RECORD_PID 2>/dev/null || true

kill -INT -$LAUNCH_PID 2>/dev/null || true
wait $LAUNCH_PID 2>/dev/null || true

# =====================
# 合并
# =====================
echo "[6/7] Merge bag..."
python3 fillback/merge_bag.py \
  --base "$INPUT_BAG" \
  --replay "$OUTPUT_BAG" \
  --out "$MERGED_BAG" \
  --override-topics "${RECORD_TOPICS[@]}"

echo "[done] merged bag: $MERGED_BAG"
