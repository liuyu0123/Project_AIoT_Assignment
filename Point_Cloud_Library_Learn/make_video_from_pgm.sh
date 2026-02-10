#!/bin/bash

# ====== 配置 ======
PGM_DIR="/home/riba/data_lidar/Lidar_pcd_result"   # 替换为你的 PGM 文件所在目录
OUTPUT_VIDEO="/home/riba/data_lidar/river_map_video.mp4"          # 输出视频文件名
FPS=20                                      # 帧率（每秒显示多少张图）

# ====== 检查目录 ======
if [ ! -d "$PGM_DIR" ]; then
    echo "错误: 目录不存在: $PGM_DIR"
    exit 1
fi

# 切换到 PGM 目录（FFmpeg 的通配符模式要求在该目录下）
cd "$PGM_DIR" || { echo "无法进入目录"; exit 1; }

# 检查是否存在 PGM 文件
if ! ls *_river_map.pgm >/dev/null 2>&1; then
    echo "警告: 在 $PGM_DIR 中未找到 '*_river_map.pgm' 文件。"
    exit 1
fi

echo "正在合成视频，请稍候..."

# 使用 FFmpeg 合成视频
# 注意：PGM 是灰度图，FFmpeg 会自动处理
ffmpeg -framerate "$FPS" \
       -f image2 \
       -pixel_format gray16le \
       -i "%010d_river_map.pgm" \
       -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2, format=yuv420p" \
       -c:v libx264 \
       -crf 23 \
       "$OUTPUT_VIDEO"

echo "✅ 视频已生成: $OUTPUT_VIDEO"