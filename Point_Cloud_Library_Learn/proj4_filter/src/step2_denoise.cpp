// src/step2_denoise.cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> <output.pcd>" << std::endl;
        return -1;
    }

    // 定义点云类型（使用标准PointXYZ）
    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    // 1. 加载点云
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        std::cerr << "Error: Could not read file " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points from " << argv[1] << std::endl;

    // 2. 去噪：统计滤波器
    PointCloudT::Ptr cloud_filtered(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);               // 邻域点数
    sor.setStddevMulThresh(1.0);    // 标准差倍数
    sor.filter(*cloud_filtered);

    std::cout << "After filtering: " << cloud_filtered->size() << " points." << std::endl;

    // 3. 保存结果
    pcl::io::savePCDFileBinary(argv[2], *cloud_filtered);
    std::cout << "Saved filtered point cloud to " << argv[2] << std::endl;

    return 0;
}