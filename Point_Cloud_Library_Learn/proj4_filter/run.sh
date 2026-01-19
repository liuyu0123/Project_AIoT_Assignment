# 创建构建目录
mkdir build && cd build

# 配置项目（会自动查找 PCL）
cmake ..

# 编译
make -j4

# 运行（假设你已有一个 PCD 文件，比如 0000000000.pcd）
pcd_raw=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/0000000000.pcd
pcd_filtered=/home/riba/GitProject/LIUYU/Point_Cloud_Library_Learn/data/step2_filtered.pcd
./step2_denoise $pcd_raw $pcd_filtered