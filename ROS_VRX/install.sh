############################# step1. 装机必装软件 ###############################
sudo apt install -y git gedit nano pip


########################## step2. 安装docker【需科学上网】 ########################
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

sudo usermod -aG docker $USER
newgrp docker          # 立即生效，无需重启


############################### step3. 安装rocker ##############################
#pip3 install --user rocker
# ubuntu24需要加后缀
pip3 install --user rocker --break-system-packages
export PATH=$PATH:~/.local/bin   # 如果 which rocker 还找不到，就加这一行


############################ step4. 安装NVIDIA驱动 ############################
sudo apt update
sudo ubuntu-drivers install   # 安全装 NVIDIA
#sudo reboot 安装完后需要重启
#nvidia-smi # 检验


####################### step5. 安装 NVIDIA Container Toolkit #####################
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk --version # 检查是否安装成功


############################ step6. 配置docker走代理 ############################3
# 1. 创建 systemd 配置文件夹
sudo mkdir -p /etc/systemd/system/docker.service.d
# 2. 新建代理配置文件
sudo nano /etc/systemd/system/docker.service.d/http-proxy.conf
# 文件内容如下：
[Service]
Environment="HTTP_PROXY=http://192.168.137.1:33210"
Environment="HTTPS_PROXY=http://192.168.137.1:33210"
Environment="NO_PROXY=localhost,127.0.0.1"
# 3. 重载并重启 Docker
sudo systemctl daemon-reload
sudo systemctl restart docker
# 4. 验证是否已读入代理
sudo systemctl show --property=Environment docker
# 5. 再拉镜像
# 22.04
#docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi
# 24.04
docker run --rm --gpus all nvidia/cuda:12.9.1-base-ubuntu24.04 nvidia-smi


########################### step7. 拉取 VRX 开发环境 ############################
#rocker --pull --devices /dev/input/js0 --x11 --nvidia --user --home \
#       ghcr.io/osrf/vrx-devel:latest /bin/bash
# 上面那条指令进入容器后找不到显卡，用下面这个：
rocker --pull --devices /dev/dri --x11 --nvidia --user --home ghcr.io/osrf/vrx-devel:latest /bin/bash
# 不确定 Step 3/6 : FROM ghcr.io/osrf/vrx-devel:latest 是否卡住，开新终端：
#docker pull ghcr.io/osrf/vrx-devel:latest
# docker自带保护，不用担心冲突。

# 进入容器后，验证显卡驱动
sudo apt update && sudo apt install -y mesa-utils
nvidia-smi
echo $DISPLAY
glxinfo | grep -E "(OpenGL renderer|NVIDIA)"
glxinfo | grep "OpenGL renderer"


########################### step8. 容器仿真环境配置 ############################
# 1. 创建一个 colcon 工作区并克隆 vrx 仓库
mkdir -p ~/vrx_ws/src
cd ~/vrx_ws/src
git clone https://github.com/osrf/vrx.git
# 2. 获取您的 ROS 2 安装环境。
source /opt/ros/jazzy/setup.bash
# 3. 构建工作区
cd ~/vrx_ws
colcon build --merge-install
#  现在你已经构建了仿真环境，需要在使用它之前运行设置脚本。从工作空间的根目录开始，执行以下命令
. install/setup.bash


########################### step9. 容器内启动仿真 ############################
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
# 执行后应出现图形界面


