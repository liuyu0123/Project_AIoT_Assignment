# 创建构建目录
if [ ! -d "build" ]; then
  echo "build已存在，跳过创建..."
  mkdir build
fi
cd build

# 配置项目（会自动查找 PCL）
cmake ..

# 编译
make -j4

# 点云去噪
pcd_raw=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/0000000000.pcd
pcd_filtered=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/step2_filtered.pcd
./step2_denoise $pcd_raw $pcd_filtered


# 点云分割：水面/河岸
pcd_water=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/step3_water.pcd
pcd_shore=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/step3_shore.pcd
./step3_segment_water $pcd_raw $pcd_water $pcd_shore
# 可视化水域
# pcl_viewer step3_water.pcd -fc 255,255,255 -ps 4
# 可视化河岸
# pcl_viewer step3_shore.pcd -fc 255,0,0 -ps 4
# 可视化both
# pcl_viewer step3_water.pcd -fc 255,255,255 step3_shore.pcd -fc 255,0,0 -ps 2
# pcl_viewer step3_water.pcd step3_shore.pcd


# 点云聚类
pcd_clusters=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/step4_shore_clusters.pcd
./step4_cluster_shore $pcd_shore $pcd_clusters
# pcl_viewer step3_shore.pcd -fc 255,255,255 -ps 4
# pcl_viewer step4_shore_clusters.pcd -fc 255,255,255 -ps 4

# 可视化
# CloudCompare step4_shore_clusters.pcd
