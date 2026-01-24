#!/usr/bin/env bash
set -e

INPUT_BAG=$1
TS=$(date +%Y%m%d_%H%M%S)
RECORD_BAG="${INPUT_BAG%.*}_fillback_${TS}"
MERGED_BAG="${INPUT_BAG%.*}_merged_${TS}"

RECORD_TOPICS=(
  /debug/fused/objects_markers
)

READY_TOPIC=/fused/objects
LAUNCH_CMD="ros2 launch welaboat_bringup fillback.launch.py"

if [ -z "$INPUT_BAG" ]; then
  echo "Usage: $0 <input_bag>"
  exit 1
fi

source install/setup.bash

echo "[1/6] Launch fillback nodes (use_sim_time)..."
eval "$LAUNCH_CMD &"
LAUNCH_PID=$!

wait_for_topic () {
  local topic=$1
  local timeout=${2:-30}
  local start=$(date +%s)
  while ! ros2 topic list | grep -Fx "$topic" >/dev/null; do
    sleep 0.5
    if (( $(date +%s) - start > timeout )); then
      echo "Timeout waiting for $topic"
      exit 1
    fi
  done
}

echo "[2/6] Wait for /clock subscriber..."
wait_for_topic /clock 30

echo "[3/6] Start recording fillback topics..."
ros2 bag record "${RECORD_TOPICS[@]}" -o "$RECORD_BAG" --storage mcap &
REC_PID=$!

sleep 1

echo "[4/6] Play input bag with clock..."
ros2 bag play "$INPUT_BAG" --clock

echo "[5/6] Stop recording..."
kill -INT $REC_PID
wait $REC_PID || true

kill -INT -$LAUNCH_PID
wait $LAUNCH_PID || true

echo "[6/6] Merge bags..."
python3 fillback/merge_bag.py \
  --base "$INPUT_BAG" \
  --replay "$RECORD_BAG" \
  --out "$MERGED_BAG" \
  --override-topics "${RECORD_TOPICS[@]}"

echo "✅ DONE: $MERGED_BAG"
