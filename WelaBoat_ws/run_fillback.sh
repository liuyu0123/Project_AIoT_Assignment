old_bag=old.db3
new_debug_bag=new_debug_bag

# 先启动节点，在开终端运行本脚本

ros2 bag play $old_bag --clock &
ros2 bag record \
  /fused/freespace_debug \
  /fused/objects_debug \
  /fused/objects_markers_debug \
  -o $new_debug_bag
