# YOLOv5双目目标检测与测距完整方案
基于搜索结果，我为您整理了一套完整的YOLOv5双目测距方案，包含相机标定、立体匹配、目标检测和距离计算全流程代码。

这套方案适用于 Python 3.8-3.11（推荐使用Python 3.8或3.9，兼容性最佳）。


# 一、方案架构
## 双目测距的核心流程：
1. 双目相机标定：获取相机内外参数
2. 图像校正与立体匹配：生成视差图
3. YOLOv5目标检测：识别目标边界框
4. 深度计算：结合视差图计算目标距离
5. 结果可视化：标注距离信息


# 二、环境配置
```bash
# 创建虚拟环境
conda create -n yolov5_stereo python=3.8
conda activate yolov5_stereo

# 安装依赖
pip install torch torchvision opencv-python numpy scipy PyQt5
pip install matplotlib open3d  # 可选，用于可视化

# 下载YOLOv5
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip install -r requirements.txt
cd ..
```

# 三、代码实现
1. 相机参数配置：camera_config.py
2. 深度计算工具：dis_count.py
3. 主程序：stereo_yolov5.py
4. 相机标定脚本：camera_calibration.py


# 四、使用说明
1. 相机标定
运行标定脚本，拍摄10-20组不同角度的棋盘格图片：

2. 更新相机参数
- 将标定得到的参数替换到 camera_config.py 中：
- camera_matrix_left 和 camera_matrix_right
- dist_coeffs_left 和 dist_coeffs_right
- R 和 T 矩阵

3. 运行主程序

4. 参数调整
- 修改 cap_left = cv2.VideoCapture(0) 和 cap_right = cv2.VideoCapture(2) 中的设备号
- 调整 model.conf 和 model.iou 改变检测灵敏度
- 修改 min_disp 和 num_disp 优化深度图质量


# 五、关键改进点
相比单目测距，双目方案的优势：
- 无需训练深度模型：直接通过立体匹配计算真实深度
- 更高精度：特别是近距离测量（0.5-10米）
- 实时性好：现代GPU可实时处理
- 鲁棒性强：不受物体类别限制


# 注意事项：
- 确保双目相机同步采集
- 基线距离建议在50-100mm之间
- 光照条件影响立体匹配效果
- 近距离物体（<0.5m）可能无法匹配


# 六、扩展功能
如需更高精度，可替换立体匹配算法：
```python
# 使用RAFT-Stereo等深度学习匹配算法
# pip install raft-stereo
from raft_stereo import RAFTStereo

class AdvancedDepthEstimator(DepthEstimator):
    def __init__(self, camera_config):
        super().__init__(camera_config)
        self.model = torch.hub.load('github.com/princeton-vl/RAFT', 'raft_stereo')
        self.model.eval()
```



