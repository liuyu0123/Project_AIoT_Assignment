#!/bin/bash

# ====== 参数检查 ======
if [ "$#" -ne 1 ]; then
    echo "用法: $0 <pcd文件所在目录>"
    echo "示例: $0 /home/riba/data_lidar/Lidar_pcd"
    exit 1
fi

PCD_DIR="$1"

# 检查目录是否存在
if [ ! -d "$PCD_DIR" ]; then
    echo "错误: 目录不存在: $PCD_DIR"
    exit 1
fi

# 确保路径是绝对路径（可选但推荐）
PCD_DIR=$(realpath "$PCD_DIR")

# ====== 配置 run_input.sh 路径 ======
# 请根据实际情况修改这一行！
RUN_SCRIPT="./run_input.sh"   # 如果在当前目录
# RUN_SCRIPT="/path/to/your/run_input.sh"  # 或使用绝对路径

if [ ! -f "$RUN_SCRIPT" ]; then
    echo "错误: 找不到处理脚本: $RUN_SCRIPT"
    exit 1
fi

if [ ! -x "$RUN_SCRIPT" ]; then
    echo "警告: $RUN_SCRIPT 不可执行，尝试添加执行权限..."
    chmod +x "$RUN_SCRIPT" || { echo "失败，请手动运行 'chmod +x $RUN_SCRIPT'"; exit 1; }
fi

# ====== 获取所有 .pcd 文件 ======
shopt -s nullglob  # 避免无匹配时返回 "*.pcd" 字面量
pcd_files=("$PCD_DIR"/*.pcd)
shopt -u nullglob

if [ "${#pcd_files[@]}" -eq 0 ]; then
    echo "警告: 在 $PCD_DIR 中未找到任何 .pcd 文件。"
    exit 0
fi

total=${#pcd_files[@]}
echo "共找到 $total 个 .pcd 文件，开始批量处理..."

# ====== 遍历并处理 ======
counter=1
for pcd in "${pcd_files[@]}"; do
    if [ ! -f "$pcd" ]; then
        continue  # 跳过无效项（理论上不会发生）
    fi

    filename=$(basename "$pcd")
    echo "[$counter/$total] 正在处理: $filename"
    
    # 调用你的处理脚本
    "$RUN_SCRIPT" "$pcd"
    
    ((counter++))
done

echo "✅ 所有 .pcd 文件处理完成！"