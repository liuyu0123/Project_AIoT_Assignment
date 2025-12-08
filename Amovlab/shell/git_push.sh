#!/bin/bash
# 文件：loop_push.sh
# cd ~/LIUYU/Amovlab      # ① 进入仓库目录
while :; do
    git push origin "$(git rev-parse --abbrev-ref HEAD)"  # ② 推当前分支
    sleep 120        # ③ 120 秒 = 2 分钟
done