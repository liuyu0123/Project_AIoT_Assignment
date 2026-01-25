#!/bin/bash

# old_bag=old.db3
# new_debug_bag=new_debug_bag
old_bag=/home/riba/GitProject/LIUYU/WelaBoat_ws/record/Target3DNew/rosbag2_2026_01_24-16_31_52/rosbag2_2026_01_24-16_31_52_0.mcap
new_debug_bag=/home/riba/GitProject/LIUYU/WelaBoat_ws/record/record1

BAG=$old_bag
OUT=$new_debug_bag

# 先启动节点，在开终端运行本脚本
echo "运行本脚本后，在打开的tmux中执行：ros2 bag play --resume"

tmux new-session -d -s fillback


# pane 0：bag play（暂停启动）
tmux send-keys -t fillback:0 \
  "ros2 bag play $BAG --clock --pause --disable-keyboard" C-m


# pane 1：record
tmux split-window -h
tmux send-keys -t fillback:0.1 \
  "ros2 bag record \
    /fused/objects_debug \
    /fused/objects_markers_debug \
    /fused/freespace_debug \
    -o $OUT --storage mcap" C-m


# pane 2：你留着看 / 控制
tmux split-window -v
tmux send-keys -t fillback:0.2 "echo Ready. Resume when node is ready." C-m

tmux attach -t fillback



# ros2 bag play $old_bag --clock --disable-keyboard  &
# ros2 bag record \
#   /fused/freespace_debug \
#   /fused/objects_debug \
#   /fused/objects_markers_debug \
#   -o $new_debug_bag \
#   --storage mcap
