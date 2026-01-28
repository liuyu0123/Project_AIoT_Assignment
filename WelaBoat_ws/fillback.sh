#!/usr/bin/env bash
set -e

INPUT_CONVERT=1
ONLY_FILLBACK=1

if [ $# -lt 1 ]; then
  echo "Usage: $0 <input_bag (dir or .mcap)>"
  exit 1
fi

INPUT_BAG="$1"
WS="$HOME/GitProject/LIUYU/WelaBoat_ws"
FILLBACK="$WS/fillback"
TS=$(date +%Y%m%d_%H%M%S)

# -----------------------------
# Resolve base bag dir
# -----------------------------
if [ -f "$INPUT_BAG" ]; then
  BASE_DIR="$(dirname "$INPUT_BAG")"
else
  BASE_DIR="$INPUT_BAG"
fi

echo "[fillback] base bag dir: $BASE_DIR"

BASE_DB3="${BASE_DIR}_db3"
# FILL_DB3="${BASE_DIR}_fillback"
# MERGED_DB3="${BASE_DIR}_merged_db3"
# MERGED_MCAP="${BASE_DIR}_merged"
FILL_DB3="${BASE_DIR}_fillback_${TS}"
MERGED_DB3="${BASE_DIR}_merged_db3_${TS}"
MERGED_MCAP="${BASE_DIR}_merged_${TS}"

# -----------------------------
# YAMLs
# -----------------------------
DB3_YAML="/tmp/convert_to_db3_${TS}.yaml"
MCAP_YAML="/tmp/convert_to_mcap_${TS}.yaml"

cat > "$DB3_YAML" <<EOF
output_bags:
  - uri: $BASE_DB3
    storage_id: sqlite3
EOF

cat > "$MCAP_YAML" <<EOF
output_bags:
  - uri: $MERGED_MCAP
    storage_id: mcap
EOF




colcon build
source ~/miniconda3/bin/activate yolov5_env
python -m colcon build --packages-select yolov5_detector fastscnn_segmenter --symlink-install
conda deactivate
conda deactivate
source install/setup.bash






# -----------------------------
# Step 0: normalize base bag
# -----------------------------
if [ ! -d "$BASE_DB3" ]; then
  echo "[0/7] Convert base bag to sqlite3..."
  ros2 bag convert -i "$BASE_DIR" -o "$DB3_YAML"
fi

# -----------------------------
# Step 1: launch nodes
# -----------------------------
echo "[1/7] Launch fillback nodes..."
ros2 launch welaboat_bringup fillback.launch.py use_sim_time:=true &
LAUNCH_PID=$!
sleep 3

# -----------------------------
# Step 2: record fillback
# -----------------------------
echo "[2/7] Record fillback topics..."
ros2 bag record \
  /debug/fused/objects_markers \
  --use-sim-time \
  -o "$FILL_DB3" &
REC_PID=$!
sleep 2

# -----------------------------
# Step 3: play base bag
# -----------------------------
echo "[3/7] Play base bag..."
ros2 bag play "$BASE_DB3" --clock -r 0.3

# -----------------------------
# Step 4: stop record
# -----------------------------
kill $REC_PID
wait $REC_PID || true

# -----------------------------
# Step 5: stop nodes
# -----------------------------
kill $LAUNCH_PID
wait $LAUNCH_PID || true


if [ $ONLY_FILLBACK -eq 1 ]; then
  echo "Skip merging..."
  echo "✅ DONE"
  exit 0
fi


# -----------------------------
# Step 6: merge db3
# -----------------------------
echo "[6/7] Merge bags..."
python3 "$FILLBACK/merge_bag.py" \
  --base "$BASE_DB3" \
  --replay "$FILL_DB3" \
  --out "$MERGED_DB3" \
  --override-topics /debug/fused/objects_markers

# -----------------------------
# Step 7: convert to mcap
# -----------------------------
echo "[7/7] Convert merged bag to mcap..."
ros2 bag convert -i "$MERGED_DB3" -o "$MCAP_YAML"

echo
echo "✅ DONE"
echo "👉 Final bag: $MERGED_MCAP"
