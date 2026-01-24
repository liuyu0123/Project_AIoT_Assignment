#!/usr/bin/env bash
set -e

INPUT_BAG=$1
TS=$(date +%Y%m%d_%H%M%S)
RECORD_BAG="${INPUT_BAG%.*}_fillback_${TS}"
MERGED_BAG="${INPUT_BAG%.*}_merged_${TS}"

RECORD_TOPICS=(
  /debug/fused/objects_markers
)

LAUNCH_CMD="ros2 launch welaboat_bringup fillback.launch.py"

source install/setup.bash

echo "[1/6] Launch fillback nodes (use_sim_time)..."
eval "$LAUNCH_CMD &"
LAUNCH_PID=$!

echo "[2/6] Start recording fillback topics..."
ros2 bag record "${RECORD_TOPICS[@]}" -o "$RECORD_BAG" --storage mcap &
REC_PID=$!

sleep 2

echo "[3/6] Play input bag with clock (slow rate)..."
ros2 bag play "$INPUT_BAG" \
  --clock \
  --rate 0.3 \
  --read-ahead-queue-size 1000 &
PLAY_PID=$!

wait $PLAY_PID

echo "[4/6] Stop recording..."
kill -INT $REC_PID
wait $REC_PID || true

echo "[5/6] Stop fillback nodes..."
pkill -P $LAUNCH_PID || true
kill $LAUNCH_PID || true
wait $LAUNCH_PID || true

echo "[6/6] Merge bags..."
python3 fillback/merge_bag.py \
  --base "$INPUT_BAG" \
  --replay "$RECORD_BAG" \
  --out "$MERGED_BAG" \
  --override-topics "${RECORD_TOPICS[@]}"

echo "✅ DONE: $MERGED_BAG"
