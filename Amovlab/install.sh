set -e
python_edition=python3 #python3.9
python_venv_name=dronekit-env

if [ $python_edition -eq "python3.9" ];then
python3.9 -m venv dronekit-env-py39   # 用 3.9 创建新环境
source dronekit-env-py39/bin/activate
python3.9 -m pip install dronekit
python3.9 -m pip install pymavlink
python3.9 -m pip install future
python3.9 -m pip install --upgrade pymavlink
python3.9 -m pip install numpy opencv-python
#python3.9 -m pip list

else
$python_edition -m venv $python_venv_name
source $python_venv_name/bin/activate
$python_edition -m pip install dronekit
$python_edition -m pip install pymavlink
$python_edition -m pip install future
$python_edition -m pip install --upgrade pymavlink
$python_edition -m pip install numpy opencv-python
#$python_edition -m pip list
fi