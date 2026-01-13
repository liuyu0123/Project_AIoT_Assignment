colcon build
source install/setup.bash
# 启动相机驱动节点
ros2 run camera_driver camera_driver
# 启动相机预览节点
ros2 run camera_driver camera_viewer /camera/left/image_raw


