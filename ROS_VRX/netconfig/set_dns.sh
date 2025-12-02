#!/usr/bin/env bash
# set-dns-and-test.sh
# 为无线网卡 wlp7s0 设置 DNS 并简单测试

set -euo pipefail

IF="wlp7s0"          # 无线网卡名，按需修改
DNS1="8.8.8.8"
DNS2="1.1.1.1"
TEST_DOM="baidu.com"

echo "==> 1/4 设置 DNS 服务器"
sudo resolvectl dns    "$IF" "$DNS1" "$DNS2"

echo "==> 2/4 设置搜索域（~. 表示全局）"
sudo resolvectl domain "$IF" "~."

echo "==> 3/4 dig 解析 $TEST_DOM"
dig "$TEST_DOM" +short

echo "==> 4/4 ping 测试 $TEST_DOM"
ping -c 3 "$TEST_DOM"

echo "==> 全部完成！"