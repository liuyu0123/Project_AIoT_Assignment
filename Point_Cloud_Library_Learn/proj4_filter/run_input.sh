#!/bin/bash
set -e

# 创建构建目录
if [ ! -d "build" ]; then
    echo "build 不存在，正在创建..."
    mkdir build
else
    echo "build 已存在，跳过创建..."
fi

cd build

# 配置并编译
# cmake ..
# make -j4

# 获取输入文件路径（从命令行参数）
pcd_raw="$1"
if [ -z "$pcd_raw" ]; then
    echo "用法: $0 <input.pcd>"
    exit 1
fi

pcd_raw_path=$(realpath "$pcd_raw")
pcd_raw_name=$(basename "$pcd_raw_path" .pcd)  # 去掉 .pcd 后缀
raw_path=$(dirname "$pcd_raw_path")
output_path=$(dirname "$pcd_raw_path")_result
output_pcd_raw_name="${output_path}/${pcd_raw_name}"

# Step2: 去噪
pcd_filtered="${output_pcd_raw_name}_step2_filtered.pcd"
./step2_denoise "$pcd_raw" "$pcd_filtered"

# Step3: 分割水面/河岸
pcd_water="${output_pcd_raw_name}_step3_water.pcd"
pcd_shore="${output_pcd_raw_name}_step3_shore.pcd"
./step3_segment_water "$pcd_filtered" "$pcd_water" "$pcd_shore"

# Step4: 河岸聚类
pcd_clusters="${output_pcd_raw_name}_step4_shore_clusters.pcd"
./step4_cluster_shore "$pcd_shore" "$pcd_clusters"

# Step5: 投影到2D
pcd_projected="${output_pcd_raw_name}_step5_projected.pcd"
./step5_project_2d "$pcd_clusters" "$pcd_projected"

# Step6: 提取左右岸线（使用方法2）
pcd_left_line="${output_pcd_raw_name}_step6_left_shore_line.pcd"
pcd_right_line="${output_pcd_raw_name}_step6_right_shore_line.pcd"
./step6_extract_centerlines2 "$pcd_projected" "$pcd_left_line" "$pcd_right_line"

# Step7: 插值补齐
./step7_interpolate_shores "$pcd_left_line" "$pcd_right_line" "${output_path}"

# Step8: 生成地图
./step8_generate_occupancy_grid \
    "${output_path}/left_interp.pcd" \
    "${output_path}/right_interp.pcd" \
    "${output_pcd_raw_name}_river_map"