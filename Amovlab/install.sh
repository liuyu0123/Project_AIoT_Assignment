set -e
board=RaspberryPi3B
sudo apt update
sudo apt install python3-pip python3-dev python3-venv -y

# python_edition=python3 #python3.9
# python_venv_name=dronekit-env

# python3 -m pip venv dronekit-env
# source dronekit-env/bin/activate

if [ $board = "RaspberryPi5" ]; then

env_name=dronekit-env-py39
if [ -d "$env_name" ]; then
echo "$env_name 开发环境已存在"
source $env_name/bin/activate
else
echo "创建 $env_name 开发环境..."
python3.9 -m venv $env_name   # 用 3.9 创建新环境
source $env_name/bin/activate
fi

pip install dronekit
pip install pymavlink
pip install future
pip install --upgrade pymavlink
pip install numpy opencv-python
pip install pyserial

elif [ $board = "RaspberryPi3B" ]; then
#树莓派3B，ubuntu20.04，系统python版本3.8
sudo apt install libgl1-mesa-glx -y

env_name=dronekit-env
if [ -d "$env_name" ]; then
echo "$env_name 开发环境已存在"
source $env_name/bin/activate
else
echo "创建 $env_name 开发环境..."
python3 -m venv $env_name   # 用 3.9 创建新环境
source $env_name/bin/activate
fi

pip install wheel -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install lxml --only-binary=lxml -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install dronekit -i https://pypi.tuna.tsinghua.edu.cn/simple
# pip install pymavlink -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install pymavlink==2.4.48 -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install future -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install --upgrade pymavlink
pip install numpy opencv-python -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install pyserial -i https://pypi.tuna.tsinghua.edu.cn/simple
pip pigpio

else
echo "开发板名称错误，请检查..."
fi