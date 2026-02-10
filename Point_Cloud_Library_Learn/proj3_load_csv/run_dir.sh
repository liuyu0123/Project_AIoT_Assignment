#!/bin/bash

# ====== 配置区 ======
CSV_DIR="/home/riba/data_lidar/Lidar"          # CSV 文件所在目录
PCD_DIR="/home/riba/data_lidar/Lidar_pcd"      # PCD 输出目录
SCRIPT_PATH="csv2pcd.py"                       # 确保该脚本在 PATH 中，或使用绝对路径

# 可视化参数（可选）
POINT_COLOR_WHITE="255,255,255"
POINT_SIZE=2

# ===================

# 创建输出目录（如果不存在）
mkdir -p "$PCD_DIR"

# 检查 csv2pcd.py 是否存在（如果是相对路径或固定位置，建议用绝对路径）
if [[ ! -f "$SCRIPT_PATH" ]] && ! command -v csv2pcd.py &> /dev/null; then
    echo "错误: 找不到 csv2pcd.py 脚本。请检查路径或将其加入 PATH。"
    exit 1
fi

# 获取所有 .csv 文件（按文件名排序）
shopt -s nullglob  # 避免无匹配时返回字面 "*.csv"
csv_files=("$CSV_DIR"/*.csv)
shopt -u nullglob

if [ ${#csv_files[@]} -eq 0 ]; then
    echo "警告: 在 $CSV_DIR 中未找到任何 .csv 文件。"
    exit 0
fi

total=${#csv_files[@]}
echo "共找到 $total 个 CSV 文件，开始转换..."

counter=1
for csv in "${csv_files[@]}"; do
    # 提取不带路径和扩展名的文件名，例如 /path/000001.csv → 000001
    basename_no_ext=$(basename "$csv" .csv)
    
    pcd_out="$PCD_DIR/${basename_no_ext}.pcd"
    
    echo "[$counter/$total] 转换: $(basename "$csv") → $(basename "$pcd_out")"
    
    # 执行转换
    python3 "$SCRIPT_PATH" "$csv" "$pcd_out"
    
    # 可选：自动用 pcl_viewer 查看（若不需要，注释掉下面两行）
    # echo "正在查看: $pcd_out （按 Ctrl+C 或关闭窗口继续）"
    # pcl_viewer "$pcd_out" -fc "$POINT_COLOR_WHITE" -ps "$POINT_SIZE"
    
    ((counter++))
done

echo "✅ 所有文件已转换完成，PCD 文件保存在: $PCD_DIR"