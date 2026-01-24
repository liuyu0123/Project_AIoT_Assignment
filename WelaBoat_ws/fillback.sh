#!/usr/bin/env bash
set -e

# ===============================
# fillback.sh (ULTIMATE - HUMBLE)
# ===============================

if [ $# -lt 1 ]; then
  echo "Usage: $0 <input_bag (dir or .mcap)>"
  exit 1
fi

INPUT_BAG="$1"
WORKSPACE="$HOME/GitProject/LIUYU/WelaBoat_ws"
FILLBACK_DIR="$WORKSPACE/fillback"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# -------------------------------
# Resolve base bag dir
# -------------------------------
if [ -f "$INPUT_BAG" ]; then
  BASE_BAG_DIR="$(dirname "$INPUT_BAG")"
else
  BASE_BAG_DIR="$INPUT_BAG"
fi

echo "[fillback] base bag dir: $BASE_BAG_DIR"

# -------------------------------
# Step 0: normalize base bag to db3
# -------------------------------
BASE_DB3="${BASE_BAG_DIR}_db3"

if [ ! -d "$BASE_DB3" ]; then
  echo "[0/7] Convert base bag to sqlite3..."
  ros2 bag convert \
    -i "$BASE_BAG_DIR" \
    -o "$BASE_DB3" \
    -s sqlite3
fi

# -------------------------------
# Paths
# -------------------------------
FILLBACK_BAG="${BASE_BAG_DIR}_fillback_${TIMESTAMP}"
MERGED_DB3="${BASE_BAG_DIR}_merged_db3_${TIMESTAMP}"
MERGED_MCAP="${BASE_BAG_DIR}_merged_${TIMESTAMP}"

# -------------------------------
# Step 1: launch fillback nodes
# -------------------------------
echo "[1/7] Launch fillback nodes (use_sim_time)..."
ros2 launch welaboat_bringup fillback.launch.py use_sim_time:=true &
LAUNCH_PID=$!
sleep 3

# -------------------------------
# Step 2: record fillback topics (db3 only!)
# -------------------------------
echo "[2/7] Start recording fillback topics..."
ros2 bag record \
  /debug/fused/objects_markers \
  --use-sim-time \
  -o "$FILLBACK_BAG" &
REC_PID=$!

sleep 2

# -------------------------------
# Step 3: play base bag with clock
# -------------------------------
echo "[3/7] Play input bag with clock (slow rate)..."
ros2 bag play "$BASE_DB3" --clock -r 0.3

# -------------------------------
# Step 4: stop recording
# -------------------------------
echo "[4/7] Stop recording..."
kill $REC_PID
wait $REC_PID || true

# -------------------------------
# Step 5: stop fillback nodes
# -------------------------------
echo "[5/7] Stop fillback nodes..."
kill $LAUNCH_PID
wait $LAUNCH_PID || true

# -------------------------------
# Step 6: merge db3 bags
# -------------------------------
echo "[6/7] Merge db3 bags..."
python3 "$FILLBACK_DIR/merge_bag.py" \
  --base "$BASE_DB3" \
  --replay "$FILLBACK_BAG" \
  --out "$MERGED_DB3" \
  --override-topics /debug/fused/objects_markers

# -------------------------------
# Step 7: convert merged bag to mcap
# -------------------------------
echo "[7/7] Convert merged bag to mcap..."
ros2 bag convert \
  -i "$MERGED_DB3" \
  -o "$MERGED_MCAP" \
  -s mcap

echo
echo "✅ Fillback DONE"
echo "👉 Final bag: $MERGED_MCAP"
