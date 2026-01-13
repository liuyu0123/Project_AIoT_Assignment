colcon build
source install/setup.bash
# 启动相机驱动节点
ros2 run camera_driver camera_driver
# 启动相机预览节点
ros2 run camera_driver camera_viewer /camera/left/image_raw
ros2 run camera_driver camera_viewer /camera/left/image_rect
ros2 run camera_driver camera_viewer /stereo/disparity_color  #预览视差图
# 启动相机校正节点
ros2 run camera_driver camera_rectified

# 启动双目视差节点
ros2 run stereo_disparity stereo_disparity --ros-args --params-file install/stereo_disparity/share/stereo_disparity/config/sgbm_params.yaml
# 使用launch脚本启动节点
ros2 launch stereo_disparity disparity.launch.py


# 一键启动整个链路
ros2 launch welaboat_bringup welaboat.launch.py
