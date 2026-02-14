cd record/PixHawk_GPS

ros2 bag record \
/mavros/imu/data \
/mavros/global_position/raw/fix \
/odometry/filtered \
/odometry/filtered/global \
/odometry/gps \
/tf \
/tf_static \
 --storage mcap
