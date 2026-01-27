# 1. 先安装moniconda，注意不要安装conda
cd ~
# x64_64
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
# arm64
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-aarch64.sh
bash Miniconda3-latest-Linux-aarch64.sh


# 2. 构建虚拟环境 yolov5_env
conda create -n yolov5_env -y python=3.10
conda activate yolov5_env
cd ~/yolov5
pip install -r requirements.txt
pip install colcon-common-extensions
conda deactivate


# 3. [暂时不需要]构建虚拟环境 lidar_env
conda create -n lidar_env -y python=3.10
conda activate lidar_env
pip install open3d
conda deactivate


# 4. 修复 yolov5_env 的 opencv 开发环境
# 处理 yolov5 环境下 numpy 与 opencv 版本冲突的问题
source ~/miniconda3/bin/activate yolov5_env
# step1.安装 numpy=1.X
pip install numpy==1.26.4
# step2.卸载 opencv
pip uninstall opencv-python -y
# step33. 创建软链接（关键！），将系统 opencv 软链到 conda 环境中
ln -sf /usr/lib/python3/dist-packages/cv2*.so \
       ~/miniconda3/envs/yolov5_env/lib/python3.10/site-packages/cv2.so


# 修复 yolov5_env 环境下 pytorch-cpu-only 版本
# 1. 安装torchision
source ~/miniconda3/bin/activate yolov5_env
python3 -m pip install --no-cache ~/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl
# 2. 安装opencv-python
pip install opencv-python
# 3. 安装torch
python3 -m pip install --no-cache ~/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl
# 4. 恢复numpy=1.x
pip install numpy==1.26.4
# 5. 卸载opencv-python
pip uninstall opencv-python -y
