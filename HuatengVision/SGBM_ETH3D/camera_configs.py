# author: young
import cv2
import numpy as np


# 加载图像
left_image = cv2.imread("test/test2/im0.png")
right_image = cv2.imread("test/test2/im1.png")


###########################################双目相机-内参#############################################
# 根据calib.txt和cameras.txt中的内参矩阵
left_camera_matrix = np.array([[711.415, 0, 376.739], [0, 711.415, 222.302], [0, 0, 1]])
right_camera_matrix = np.array([[711.415, 0, 376.739], [0, 711.415, 222.302], [0, 0, 1]])

# 假设畸变参数未给出，暂时设置为零畸变
left_distortion = np.zeros((4, 1), dtype=np.float32)  # k1, k2, p1, p2
right_distortion = np.zeros((4, 1), dtype=np.float32)  # k1, k2, p1, p2


###########################################双目相机-外参#############################################
# 根据calib.txt中的基线和图像尺寸
baseline = 59.9701  # 基线，单位为毫米，转换为米
baseline_m = baseline / 1000.0
image_width = 711
image_height = 435

# 假设旋转矩阵为单位矩阵（无旋转），平移向量根据基线计算
R = np.identity(3)  # 旋转矩阵
T = np.array([-baseline_m, 0, 0])  # 平移向量，假设沿X轴平移


# 立体校正
size = (image_width, image_height)  # 图像尺寸
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
    left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, size, R, T
)

# 校正查找映射表
left_map1, left_map2 = cv2.initUndistortRectifyMap(
    left_camera_matrix, left_distortion, R1, P1, size, cv2.CV_16SC2
)
right_map1, right_map2 = cv2.initUndistortRectifyMap(
    right_camera_matrix, right_distortion, R2, P2, size, cv2.CV_16SC2
)

# 提取校正后的焦距和主点
f = P1[0, 0]  # 焦距，单位为像素
cx, cy = P1[0, 2], P1[1, 2]  # 主点坐标


def show_calibrated_image():
    # 应用畸变校正和立体校正
    left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    # 显示校正后的图像
    cv2.imshow("Left Rectified", left_rectified)
    cv2.imshow("Right Rectified", right_rectified)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def show_calibrated_image_pair():
    # 确保图像加载成功
    if left_image is None or right_image is None:
        print("Error: Unable to load images. Please check the file paths.")
        exit()

    # 应用畸变校正和立体校正
    left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    # 创建画布显示原始和校正图像
    canvas_height = left_image.shape[0] * 2
    canvas_width = left_image.shape[1] * 2
    canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)

    canvas[:left_image.shape[0], :left_image.shape[1]] = left_image
    canvas[:right_image.shape[0], left_image.shape[1]:] = right_image
    canvas[left_image.shape[0]:, :left_image.shape[1]] = left_rectified
    canvas[left_image.shape[0]:, left_image.shape[1]:] = right_rectified

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(canvas, "Original Left", (10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "Original Right", (left_image.shape[1] + 10, 30), font, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "Rectified Left", (10, left_image.shape[0] + 30), font, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "Rectified Right", (left_image.shape[1] + 10, left_image.shape[0] + 30), font, 1, (255, 255, 255), 2)

    scale_factor = 0.5
    canvas_resized = cv2.resize(canvas, (0, 0), fx=scale_factor, fy=scale_factor)
    cv2.imshow("Image Comparison", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return left_rectified, right_rectified


def sgbm_show(left_image, right_image):
    blockSize = 11
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 9,
        blockSize=blockSize,
        P1=8 * 3 * blockSize ** 2,
        P2=32 * 3 * blockSize ** 2,
        disp12MaxDiff=1,
        preFilterCap=63,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )
    disparity = sgbm.compute(left_image, right_image).astype(np.float32) / 16.0
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    cv2.imshow("Disparity", disparity_normalized)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def wls_show(left_image, right_image):
    left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

    blockSize = 11
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 9,
        blockSize=blockSize,
        P1=8 * 3 * blockSize ** 2,
        P2=32 * 3 * blockSize ** 2,
        disp12MaxDiff=1,
        preFilterCap=63,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32
    )
    disparity = sgbm.compute(left_gray, right_gray).astype(np.float32) / 16.0

    wls_filter = cv2.ximgproc.createDisparityWLSFilter(sgbm)
    right_matcher = cv2.ximgproc.createRightMatcher(sgbm)
    disparity_right = right_matcher.compute(right_gray, left_gray).astype(np.float32) / 16.0

    wls_filter.setLambda(1000.0)
    wls_filter.setSigmaColor(1.0)
    filtered_disparity = wls_filter.filter(disparity, left_gray, None, disparity_right)

    disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    filtered_normalized = cv2.normalize(filtered_disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    canvas = np.hstack((disparity_normalized, filtered_normalized))
    cv2.putText(canvas, "Original Disparity", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "WLS Filtered Disparity", (left_gray.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    scale_factor = 0.5
    canvas_resized = cv2.resize(canvas, (0, 0), fx=scale_factor, fy=scale_factor)
    cv2.imshow("Disparity Comparison (Original vs WLS Filtered)", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return filtered_disparity


if __name__ == '__main__':
    print('Q = \n', Q)
    print('b = ', baseline_m)
    print('f = ', f)
    print('cx = ', cx)
    print('cy = ', cy)
    left_rectified, right_rectified = show_calibrated_image_pair()
    wls_show(left_rectified, right_rectified)