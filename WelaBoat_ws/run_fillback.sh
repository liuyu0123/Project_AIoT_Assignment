# old_bag=old.db3
# new_debug_bag=new_debug_bag
old_bag=/home/riba/GitProject/LIUYU/WelaBoat_ws/record/Target3DNew/rosbag2_2026_01_24-16_31_52/rosbag2_2026_01_24-16_31_52_0.mcap
new_debug_bag=/home/riba/GitProject/LIUYU/WelaBoat_ws/record/record1



# 先启动节点，在开终端运行本脚本

ros2 bag play $old_bag --clock --disable-keyboard  &
ros2 bag record \
  /fused/freespace_debug \
  /fused/objects_debug \
  /fused/objects_markers_debug \
  -o $new_debug_bag \
  --storage mcap
