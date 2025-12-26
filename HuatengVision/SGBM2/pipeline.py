# author: young
import cv2
import numpy as np
from camera_configs_auto import CamConf
import pdb


# 加载图像
# parent_path = r'D:\AIoT\HuatengVision\SGBM\test1'
# left_image = cv2.imread(parent_path+"\\left_003.png")
# right_image = cv2.imread(parent_path+"\\right_003.png")
left_path = r"D:\Files\Data\ImageStereo\Data3\test\left\001.png"
right_path = r"D:\Files\Data\ImageStereo\Data3\test\right\001.png"
left_image = cv2.imread(left_path)
right_image = cv2.imread(right_path)
if left_image is None or right_image is None:
    print("Error: Unable to load images. Please check the file paths.")
    exit()

# 获取相机参数
left_map1, left_map2 = CamConf['left_map1'], CamConf['left_map2']
right_map1, right_map2 = CamConf['right_map1'], CamConf['right_map2']
b, f, cx, cy = CamConf['b'], CamConf['f'], CamConf['cx'], CamConf['cy']


def reshape_img(canvas, scale_factor):
    # scale_factor = 0.5  # 缩放比例，可以根据需要调整
    canvas_resized = cv2.resize(canvas, (0, 0), fx=scale_factor, fy=scale_factor)
    return canvas_resized


def show_calibrated_image():
    # 应用畸变校正和立体校正
    left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    # 显示校正后的图像
    cv2.imshow("Left Rectified", reshape_img(left_rectified, scale_factor=0.5))
    cv2.imshow("Right Rectified", reshape_img(right_rectified, scale_factor=0.5))
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def show_calibrated_image_pair(if_show):
    # 应用畸变校正和立体校正
    left_rectified = cv2.remap(left_image, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_image, right_map1, right_map2, cv2.INTER_LINEAR)

    if if_show:
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

        # 显示缩放后的画布
        cv2.imshow("Image Comparison", reshape_img(canvas, scale_factor=0.5))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return left_rectified, right_rectified


def sgbm_show(left_image, right_image, num, blockSize, if_show):
    # 将校正后的图像转换为灰度图
    left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
    # 创建 SGBM 对象
    # blockSize = 16
    # num = 25
    # sgbm = cv2.StereoSGBM_create(
    #     minDisparity=0,
    #     numDisparities=16 * num,  # 根据焦距和基线调整
    #     blockSize=blockSize,
    #     P1=8 * 3 * blockSize ** 2,
    #     P2=32 * 3 * blockSize ** 2,
    #     disp12MaxDiff=1,
    #     preFilterCap=63,
    #     uniquenessRatio=10,
    #     speckleWindowSize=100,
    #     speckleRange=32
    # )
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=16*num, blockSize=blockSize)
    # 计算视差图并转换视差图格式（SGBM输出为16位有符号整数，需要转换为32位浮点数）
    disparity = sgbm.compute(left_image, right_image).astype(np.float32) / 16.0
    # 归一化视差图
    disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    disparity_color = cv2.applyColorMap(disparity_normalized, 2)

    if if_show:
        # 显示视差图
        # cv2.imshow("Disparity", reshape_img(disparity, 0.5))
        # cv2.waitKey(0)
        cv2.imshow("DisparityNormalized", reshape_img(disparity_color, 0.5))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return disparity_normalized, sgbm


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


def wls_show(left_image, right_image, disparity, sgbm, wls_lambda, wls_sigma, if_show):
    # # 将校正后的图像转换为灰度图
    left_gray = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
    right_gray = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)
   
    # 创建WLS滤波器
    # 第一个参数为8位单通道左图，第二个参数为16位单通道视差图
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(sgbm)
    
    # 计算右视差图（用于一致性检查）
    right_matcher = cv2.ximgproc.createRightMatcher(sgbm)
    disparity_right = right_matcher.compute(right_gray, left_gray)
    disparity_right = disparity_right.astype(np.float32) / 16.0
    
    # 设置WLS参数
    wls_filter.setLambda(wls_lambda)  # 平滑度权重，值越大越平滑
    wls_filter.setSigmaColor(wls_sigma)  # 颜色相似性权重，值越大对颜色差异越不敏感
    
    # 应用WLS滤波
    filtered_disparity = wls_filter.filter(disparity, left_gray, None, disparity_right)
    
    # 归一化原始视差图和滤波后视差图用于显示
    disparity_normalized = cv2.normalize(disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    filtered_normalized = cv2.normalize(filtered_disparity, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    # 增加色彩
    disparity_color = cv2.applyColorMap(disparity_normalized, 2)
    filtered_color = cv2.applyColorMap(filtered_normalized, 2)
    
    # 创建画布显示原始和滤波后的视差图
    canvas = np.hstack((disparity_color, filtered_color))
    cv2.putText(canvas, "Original Disparity", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(canvas, "WLS Filtered Disparity", (left_gray.shape[1] + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    if if_show:
        # 显示结果
        cv2.imshow("Disparity Comparison (Original vs WLS Filtered)", reshape_img(canvas, 0.5))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    return filtered_disparity


def creatp_output(vertices, filename):
    ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''
    with open(filename, 'w') as f:
        f.write(ply_header%dict(vert_num=len(vertices)))
        np.savetxt(f, vertices, '%f %f %f %d %d %d')


def pointcloud_show(disparity):
    import open3d as o3d
    if left_image.shape[0] != right_image.shape[0]:
        raise ValueError("左右图像高度不一致，无法合并")
    img = np.hstack((left_image, right_image))
    img_color = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    output_points = np.zeros((CamConf['size'][0]*CamConf['size'][1], 6))
    i = 0
    for row in range(disparity.shape[0]):
        for col in range(disparity.shape[1]):
            # if(disparity[row][col] != 0 and disparity[row][col] != (-16) and disparity[row][col]>1100 and disparity[row][col]<1570):
            if(disparity[row][col] != 0 and disparity[row][col] != (-16)):
                output_points[i][0] = 16*b*(col-cx)/disparity[row][col]
                output_points[i][1] = 16*b*(row-cy)/disparity[row][col]
                output_points[i][2] = 16*b*f/disparity[row][col]
                output_points[i][3] = img_color[row][col][0]
                output_points[i][4] = img_color[row][col][1]
                output_points[i][5] = img_color[row][col][2]
                i = i + 1
    output_file = 'nb.ply'
    creatp_output(output_points, output_file)
    pcd = o3d.io.read_point_cloud(output_file)
    o3d.visualization.draw_geometries([pcd])
    pass


def depthmap_show(disparity_map, focal_length, baseline, invalid_value=1):
    """
    从 SGBM 视差图生成深度图
    :param disparity_map: 视差图（二维数组，类型为 np.float32 或 np.uint8）
    :param focal_length: 相机焦距（单位：像素）
    :param baseline: 相机基线距离（单位：毫米或米）
    :param invalid_value: 用于替换无效视差值的小数值（默认为 1，值为整形，设0.1会出错）
    :return: 深度图（二维数组，类型为 np.float32）和归一化后的深度图（二维数组，类型为 np.uint8）
    """
    if_debug = False
    if if_debug:
        print("Min disparity:", np.min(disparity_map))
        print("Max disparity:", np.max(disparity_map))
        print("Mean disparity:", np.mean(disparity_map))
        pdb.set_trace()

    # 处理无效视差值
    valid_mask = disparity_map > 0  # 有效的视差值大于 0
    valid_mask = (disparity_map > 0) & (disparity_map != -16) & (disparity_map != 0)
    valid_mask = (disparity_map > 5) & (disparity_map < 255)
    disparity_map[~valid_mask] = invalid_value  # 将无效值替换为一个非常小的值

    if if_debug:
        print("Valid mask:", valid_mask)
        print("Fixed Min disparity:", np.min(disparity_map))
        print("Fixed Max disparity:", np.max(disparity_map))
        print("Fixed Mean disparity:", np.mean(disparity_map))

    # 计算深度图
    depth_map = (focal_length * baseline) / disparity_map

    # 归一化深度图以便可视化
    depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # 显示深度图
    cv2.imshow("SGDB (Normalized)", reshape_img(disparity_map, 0.5))
    cv2.imshow("Depth Map (Normalized)", reshape_img(depth_map_normalized, 0.5))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return depth_map, depth_map_normalized


def getDepthMapWithQ(disparityMap: np.ndarray, Q: np.ndarray) -> np.ndarray:
    points_3d = cv2.reprojectImageTo3D(disparityMap, Q)
    depthMap = points_3d[:, :, 2]
    reset_index = np.where(np.logical_or(depthMap < 0.0, depthMap > 65535.0))
    depthMap[reset_index] = 0
    cv2.imshow("SGDB (Normalized)", reshape_img(disparityMap, 0.5))
    cv2.imshow("Depth Map (Normalized)", reshape_img(depthMap, 0.5))
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return depthMap.astype(np.float32)



if __name__ == '__main__':
    # show_calibrated_image()
    left_rectified, right_rectified = show_calibrated_image_pair(if_show=True)
    disparity, sgbm = sgbm_show(left_rectified, right_rectified, 9, 10, if_show=True)
    # filtered_disparity = wls_show(left_rectified, right_rectified, disparity, sgbm, 1500, 1.5)
    # pointcloud_show(disparity)
    # pointcloud_show(filtered_disparity)
    # sgbm_show_debug(left_rectified, right_rectified)
    # depth_map, depth_map_normalized = depthmap_show(disparity, f, b)
    depthMap = getDepthMapWithQ(disparity, CamConf['Q'])