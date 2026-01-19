// src/step5_project_2d_labeled.cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] 
                  << " <input_labeled.pcd> <output_2d_labeled.pcd>" << std::endl;
        return -1;
    }

    // 使用带 label 的点类型
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);
    
    if (pcl::io::loadPCDFile<pcl::PointXYZL>(argv[1], *cloud) == -1)
    {
        std::cerr << "Error: Could not read file " << argv[1] << std::endl;
        return -1;
    }

    // 投影到 XY 平面：只设 Z=0，保留 label！
    for (auto& point : cloud->points)
    {
        point.z = 0.0f;  // ← 只改 Z，label 不动
    }

    // 设置点云尺寸（避免 PCL 错误）
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    // 保存为 ASCII（兼容性好）
    pcl::io::savePCDFileASCII(argv[2], *cloud);
    std::cout << "Projected " << cloud->size() << " labeled points to XY plane." << std::endl;

    return 0;
}