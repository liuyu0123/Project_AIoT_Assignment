#!/bin/bash
set -e

echo "==> 安装依赖"
sudo apt update
sudo apt install -y build-essential python3-dev wget unzip

echo "==> 下载 pigpio 源码"
cd ~
wget -O pigpio.zip https://github.com/joan2937/pigpio/archive/master.zip
unzip pigpio.zip
cd pigpio-master

echo "==> 编译并安装"
make -j$(nproc)
sudo make install
sudo ldconfig

echo "==> 安装 Python 绑定"
# cd python
sudo python3 setup.py install

echo "==> 启动 pigpiod 守护进程"
sudo pigpiod

echo "✅ pigpio 安装完成，守护进程已启动"