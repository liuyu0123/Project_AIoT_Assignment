# cd /PATH/TO/MY/GRAND/PROJECT
mkdir build
cd build
cmake ..
make  # 生成可执行文件
# ./pcd_write_test  # 运行可执行文件，生成点云pcd文件
# pcl_viewer test_pcd.pcd