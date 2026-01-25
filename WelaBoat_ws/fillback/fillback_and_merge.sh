#!/usr/bin/env bash
set -e

if [ $# -lt 4 ]; then
  echo "Usage:"
  echo "  $0 <base_bag> <replay_bag> <merged_bag> <launch_file> [override_topics...]"
  exit 1
fi

BASE_BAG=$1
REPLAY_BAG=$2
MERGED_BAG=$3
LAUNCH_FILE=$4
shift 4
OVERRIDE_TOPICS=$@

echo "[replay] base bag    : $BASE_BAG"
echo "[replay] replay bag  : $REPLAY_BAG"
echo "[replay] merged bag  : $MERGED_BAG"
echo "[replay] override    : $OVERRIDE_TOPICS"

# 启动算法节点（必须 use_sim_time=true）
ros2 launch ${LAUNCH_FILE} use_sim_time:=true &
LAUNCH_PID=$!

sleep 2

# 开始录制新输出
ros2 bag record -o ${REPLAY_BAG} ${OVERRIDE_TOPICS} &
RECORD_PID=$!

sleep 1

# 播放原始 bag（屏蔽被覆盖 topic）
ros2 bag play ${BASE_BAG} --clock \
$(for t in $OVERRIDE_TOPICS; do echo "--remap $t:=$t/_disabled"; done)

# play 结束后停止 record
kill -INT $RECORD_PID
wait $RECORD_PID

# 关闭算法节点
kill -INT $LAUNCH_PID
wait $LAUNCH_PID || true

# 合并 bag
python3 merge_bag.py \
  --base ${BASE_BAG} \
  --replay ${REPLAY_BAG} \
  --out ${MERGED_BAG} \
  --override-topics ${OVERRIDE_TOPICS}

echo "[done] merged bag ready: ${MERGED_BAG}"
