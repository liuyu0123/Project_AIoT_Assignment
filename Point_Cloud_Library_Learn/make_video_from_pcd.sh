#!/bin/bash

PCD_DIR="/home/riba/data_lidar/Lidar_pcd"   # ← 改成你的 PCD 目录
OUTPUT_VIDEO="/home/riba/data_lidar/pointcloud_video.mp4"
FPS=20

# 创建临时图像目录
TMP_DIR="/tmp/pcd_frames_$$"
mkdir -p "$TMP_DIR"

echo "正在渲染 PCD 到图像..."

# 遍历所有 .pcd 文件（按文件名排序）
counter=0
for pcd in "$PCD_DIR"/*.pcd; do
    if [ ! -f "$pcd" ]; then continue; fi

    # 生成 10 位序号：0000000000.png
    png_name=$(printf "%010d.png" $counter)
    python3 render_pcd_to_png.py "$pcd" "$TMP_DIR/$png_name"
    
    echo "已渲染: $(basename "$pcd") → $png_name"
    ((counter++))
done

if [ $counter -eq 0 ]; then
    echo "未找到任何 .pcd 文件"
    rm -rf "$TMP_DIR"
    exit 1
fi

echo "正在合成视频..."

# 用 FFmpeg 合成 MP4
ffmpeg -y \
       -framerate "$FPS" \
       -i "$TMP_DIR/%010d.png" \
       -c:v libx264 \
       -pix_fmt yuv420p \
       -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
       "$OUTPUT_VIDEO"

# 清理临时文件
rm -rf "$TMP_DIR"

echo "✅ 视频已生成: $OUTPUT_VIDEO"