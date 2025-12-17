import cv2
import numpy as np
import camera_configs
import open3d as o3d

# 设置立体匹配参数
num = 2
blockSize = 8
lambda_param = 2000  # WLS滤波的lambda参数，控制平滑程度
sigma_param = 1.1    # WLS滤波的sigma参数，控制边缘保持程度

b = camera_configs.b
f = camera_configs.f
cx = camera_configs.cx
cy = camera_configs.cy

# 读取左右图像
img_left = cv2.imread('test1/left_000.png')
img_right = cv2.imread('test1/right_000.png')


# 写入点云文件
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


def main():
    # 确保左右图像高度一致
    if img_left.shape[0] != img_right.shape[0]:
        raise ValueError("左右图像高度不一致，无法合并")

    # 拼接图像用于显示
    img = np.hstack((img_left, img_right))
    img_color = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # 矫正图像
    img_left_rectified = cv2.remap(img_left, camera_configs.left_map1, camera_configs.left_map2, cv2.INTER_LINEAR)
    img_right_rectified = cv2.remap(img_right, camera_configs.right_map1, camera_configs.right_map2, cv2.INTER_LINEAR)

    # 拼接矫正后的图像
    concat = cv2.hconcat([img_left_rectified, img_right_rectified])

    # 转换为灰度图
    imgL = cv2.cvtColor(img_left_rectified, cv2.COLOR_BGR2GRAY)
    imgR = cv2.cvtColor(img_right_rectified, cv2.COLOR_BGR2GRAY)

    S = cv2.StereoSGBM_create(minDisparity=0, numDisparities=16*num, blockSize=blockSize)
    dis = S.compute(imgL, imgR)

    # 归一化视差图
    dis_color = dis
    dis_color = cv2.normalize(dis_color, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    dis_color = cv2.applyColorMap(dis_color, 2)
    cv2.imshow("depth", dis_color)

    # WLS滤波
    wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=S)
    wls_filter.setLambda(lambda_param)
    wls_filter.setSigmaColor(sigma_param)

    # 使用右视图一致性检查
    S_right = cv2.StereoSGBM_create(minDisparity=0, numDisparities=16*num, blockSize=blockSize)
    dis_right = S_right.compute(imgR, imgL)
    filtered_disparity = wls_filter.filter(dis, img_left_rectified, disparity_map_right=dis_right)

    # 归一化滤波后的视差图
    filtered_disparity_color = cv2.normalize(filtered_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    # filtered_disparity_color = cv2.applyColorMap(filtered_disparity_color, 2)
    filtered_disparity_color = cv2.applyColorMap(filtered_disparity_color, cv2.COLORMAP_JET)
    cv2.imshow("filtered_depth", filtered_disparity_color)

    # 生成点云
    output_points = np.zeros((camera_configs.size[0]*camera_configs.size[1], 6))
    i = 0
    for row in range(filtered_disparity.shape[0]):
        for col in range(filtered_disparity.shape[1]):
            if(filtered_disparity[row][col] != 0 and filtered_disparity[row][col] != (-16) and filtered_disparity[row][col] > 1100 and filtered_disparity[row][col] < 1570):
                output_points[i][0] = 16*b*(col-cx)/filtered_disparity[row][col]
                output_points[i][1] = 16*b*(row-cy)/filtered_disparity[row][col]
                output_points[i][2] = 16*b*f/filtered_disparity[row][col]
                output_points[i][3] = img_color[row][col][0]
                output_points[i][4] = img_color[row][col][1]
                output_points[i][5] = img_color[row][col][2]
                i = i + 1

    output_file = 'nb.ply'
    creatp_output(output_points, output_file)
    pcd = o3d.io.read_point_cloud(output_file)
    o3d.visualization.draw_geometries([pcd])

    cv2.waitKey(0)



if __name__ == '__main__':
    main()