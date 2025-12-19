# author: young
import cv2
import numpy as np

left_image = cv2.imread("test1/left_003.png")
right_image = cv2.imread("test1/right_003.png")
# right_image = cv2.imread("test1/left_003.png")
# left_image = cv2.imread("test/left_001.png")
# right_image = cv2.imread("test/right_001.png")

###########################################双目相机-内参#############################################
# MATLAB内参矩阵(fx,fy,cx,cy):stereoParams.Camera1.K
left_camera_matrix = np.array([[1976.23, 0, 961.73], [0, 1970.96, 787.05], [0, 0, 1]])
# MATLAB径向畸变(k1,k2):stereoParams.Camera1.RadialDistortion
# MATLAB切向畸变(p1,p2):stereoParams.Camera1.TangentialDistortion
left_distortion = np.array([[-0.5042, 0.1898, 0, 0, 0]])  # k1,k2,p1,p2,k3

# MATLAB内参矩阵(fx,fy,cx,cy):stereoParams.Camera2.K
right_camera_matrix = np.array([[1983.33, 0, 1140.21], [0, 1957.40, 784.27], [0, 0, 1]])
# MATLAB径向畸变(k1,k2):stereoParams.Camera2.RadialDistortion
# MATLAB切向畸变(p1,p2):stereoParams.Camera2.TangentialDistortion
right_distortion = np.array([[-0.4225, 0.1442, 0, 0, 0]])

###########################################双目相机-外参#############################################
# MATLAB旋转矩阵R:stereoParams.PoseCamera2.R
R = np.array([[0.99791938115935275, 0.020322497918251945, -0.061187456107416979],
              [-0.020591742676888677, 0.9997808486201939, -0.0037729121131635096],
              [0.0610973717934187, 0.0050250184723540873, 0.99811916139772072]])
# MATLAB平移向量T:stereoParams.PoseCamera2.Translation
T = np.array([-138.70401869262, 0.57385249882396661, -6.5036359484209925])

# size = (640, 480)  # open windows size
size = (1280, 720)   # open windows size
# size = (1024, 768)   # open windows size
# size = (320, 240)   # open windows size
# R1:左摄像机旋转矩阵, P1:左摄像机投影矩阵, Q:重投影矩阵
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(left_camera_matrix, left_distortion,
                                                                  right_camera_matrix, right_distortion, size, R, T)

# 校正查找映射表,将原始图像和校正后的图像上的点一一对应起来
left_map1, left_map2 = cv2.initUndistortRectifyMap(left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2)
right_map1, right_map2 = cv2.initUndistortRectifyMap(right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2)

# 供外部脚本统一读取（单位：米 / 像素）
# b = abs(T[0]) * 1e-3          # 基线 m
# f = P1[0, 0]                  # 焦距 pixel
# cx = P1[0, 2]; cy = P1[1, 2]  # 主点

# 根据Bilibili教程
b = 1 / Q[3][2]
f = Q[2][3]
cx, cy = -Q[0][3], -Q[1][3]


def show_calibrated_image():
    # 加载原始图像
    # left_image = cv2.imread("test/left_001.png")
    # right_image = cv2.imread("test/left_001.png")

    # 应用畸变校正和立体校正
    left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    # 显示校正后的图像
    cv2.imshow("Left Rectified", left_rectified)
    cv2.imshow("Right Rectified", right_rectified)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def show_calibrated_image_pair():
    # 加载原始图像
    # left_image = cv2.imread("test/left_001.png")
    # right_image = cv2.imread("test/left_001.png")

    # 确保图像加载成功
    if left_image is None or right_image is None:
        print("Error: Unable to load images. Please check the file paths.")
        exit()

    # 应用畸变校正和立体校正
    left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    # 创建一个大画布，用于显示四个子图
    canvas_height = left_image.shape[0] * 2
    canvas_width = left_image.shape[1] * 2
    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

    # 将四个图像放置到画布上
    canvas[:left_image.shape[0], :left_image.shape[1]] = left_image  # 左上：原始左图
    canvas[:right_image.shape[0], left_image.shape[1]:] = right_image  # 右上：原始右图
    canvas[left_image.shape[0]:, :left_image.shape[1]] = left_rectified  # 左下：校正左图
    canvas[left_image.shape[0]:, left_image.shape[1]:] = right_rectified  # 右下：校正右图

    # 添加文字说明
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(canvas, "Original Left", (10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "Original Right", (left_image.shape[1] + 10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "Rectified Left", (10, left_image.shape[0] + 30), font, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "Rectified Right", (left_image.shape[1] + 10, left_image.shape[0] + 30), font, 1, (255, 255, 255), 2)

    # 缩放画布以适应屏幕显示
    scale_factor = 0.5  # 缩放比例，可以根据需要调整
    canvas_resized = cv2.resize(canvas, (0, 0), fx=scale_factor, fy=scale_factor)

    # 显示缩放后的画布
    cv2.imshow("Image Comparison", canvas_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return left_rectified, right_rectified


def sgbm_show(left_image, right_image):
    # 加载图像
    # left_image = cv2.imread("left_rectified.jpg", cv2.IMREAD_GRAYSCALE)
    # right_image = cv2.imread("right_rectified.jpg", cv2.IMREAD_GRAYSCALE)
    # 创建 SGBM 对象
    blockSize = 11
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 9,  # 根据焦距和基线调整
        blockSize=blockSize,
        P1=8 * 3 * blockSize ** 2,
        P2=32 * 3 * blockSize ** 2,
        disp12MaxDiff=1,
        preFilterCap=63,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )
    # 计算视差图
    disparity = sgbm.compute(left_image, right_image).astype(np.float32) / 16.0
    # 归一化视差图
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # 显示视差图
    cv2.imshow("Disparity", disparity_normalized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def sgbm_show_debug(left_image, right_image):
    # 加载图像
    # left_image = cv2.imread("left_rectified.jpg", cv2.IMREAD_GRAYSCALE)
    # right_image = cv2.imread("right_rectified.jpg", cv2.IMREAD_GRAYSCALE)
    # 定义参数范围
    block_sizes = [5, 7, 9]
    P1_P2_ratios = [(8, 32), (16, 64)]
    # 测试不同的参数组合
    for blockSize in block_sizes:
        for P1, P2 in P1_P2_ratios:
            sgbm = cv2.StereoSGBM_create(
                minDisparity=0,
                numDisparities=16 * 9,
                blockSize=blockSize,
                P1=P1 * blockSize ** 2,
                P2=P2 * blockSize ** 2,
                disp12MaxDiff=1,
                preFilterCap=63,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32
            )
            # 计算视差图
            disparity = sgbm.compute(left_image, right_image).astype(np.float32) / 16.0
            # 归一化视差图
            disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            # 显示视差图
            cv2.imshow(f"Disparity (blockSize={blockSize}, P1={P1}, P2={P2})", disparity_normalized)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


def wls_show(left_image, right_image):
    # 将校正后的图像转换为灰度图
    left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
    
    # 首先使用SGBM计算初始视差图
    blockSize = 11
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 9,
        blockSize=blockSize,
        P1=8 * 3 * blockSize **2,
        P2=32 * 3 * blockSize** 2,
        disp12MaxDiff=1,
        preFilterCap=63,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )
    disparity = sgbm.compute(left_gray, right_gray)
    
    # 转换视差图格式（SGBM输出为16位有符号整数，需要转换为32位浮点数）
    disparity = disparity.astype(np.float32) / 16.0
    
    # 创建WLS滤波器
    # 第一个参数为8位单通道左图，第二个参数为16位单通道视差图
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(sgbm)
    
    # 计算右视差图（用于一致性检查）
    right_matcher = cv2.ximgproc.createRightMatcher(sgbm)
    disparity_right = right_matcher.compute(right_gray, left_gray)
    disparity_right = disparity_right.astype(np.float32) / 16.0
    
    # 设置WLS参数
    wls_filter.setLambda(1000.0)  # 平滑度权重，值越大越平滑
    wls_filter.setSigmaColor(1.0)  # 颜色相似性权重，值越大对颜色差异越不敏感
    
    # 应用WLS滤波
    filtered_disparity = wls_filter.filter(disparity, left_gray, None, disparity_right)
    
    # 归一化原始视差图和滤波后视差图用于显示
    disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    filtered_normalized = cv2.normalize(filtered_disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    
    # 创建画布显示原始和滤波后的视差图
    canvas = np.hstack((disparity_normalized, filtered_normalized))
    cv2.putText(canvas, "Original Disparity", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "WLS Filtered Disparity", (left_gray.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    scale_factor = 0.5  # 缩放比例，可以根据需要调整
    canvas_resized = cv2.resize(canvas, (0, 0), fx=scale_factor, fy=scale_factor)
    # 显示结果
    cv2.imshow("Disparity Comparison (Original vs WLS Filtered)", canvas_resized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return filtered_disparity



if __name__ == '__main__':
    print('Q = \n', Q)
    print('b = ', b)
    print('f = ', f)
    print('cx = ', cx)
    print('cy = ', cy)
    # print(P1)
    # show_calibrated_image()
    left_rectified, right_rectified = show_calibrated_image_pair()
    # sgbm_show(left_rectified, right_rectified)
    wls_show(left_rectified, right_rectified)
    # sgbm_show_debug(left_rectified, right_rectified)