# 安装 python3.9
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.9 python3.9-venv python3.9-dev -y

python3.9 -m venv stereocam_env_py39   # 用 3.9 创建新环境
source stereocam_env_py39/bin/activate

pip install numpy
pip install open3d
pip install opencv-python
#用于wls滤波
pip install opencv-contrib-python

