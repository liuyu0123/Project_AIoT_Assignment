py -3.9 -m venv yolov5_stereo_env
yolov5_stereo_env\Scripts\Activate.ps1
pip install -r requirements.txt

git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip install -r requirements.txt