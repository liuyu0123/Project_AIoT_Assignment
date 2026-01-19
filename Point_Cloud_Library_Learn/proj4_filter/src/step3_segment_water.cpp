// src/step3_segment_water.cpp
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] 
                  << " <input.pcd> <water_output.pcd> <shore_output.pcd>" << std::endl;
        return -1;
    }

    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    // 1. 加载去噪后的点云
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        std::cerr << "Error: Could not read file " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " points." << std::endl;

    // 2. 提取水面点：Z ∈ [-0.2, 0.2]
    PointCloudT::Ptr water_cloud(new PointCloudT);
    pcl::PassThrough<PointT> pass_water;
    pass_water.setInputCloud(cloud);
    pass_water.setFilterFieldName("z");
    pass_water.setFilterLimits(-0.2f, 0.2f);
    pass_water.filter(*water_cloud);

    // 3. 提取岸上点：Z > 0.2
    PointCloudT::Ptr shore_cloud(new PointCloudT);
    pcl::PassThrough<PointT> pass_shore;
    pass_shore.setInputCloud(cloud);
    pass_shore.setFilterFieldName("z");
    pass_shore.setFilterLimits(0.2f, 100.0f); // 上限设大些
    pass_shore.filter(*shore_cloud);

    // 4. 保存结果
    pcl::io::savePCDFileBinary(argv[2], *water_cloud);
    pcl::io::savePCDFileBinary(argv[3], *shore_cloud);

    std::cout << "Water points: " << water_cloud->size() << std::endl;
    std::cout << "Shore points: " << shore_cloud->size() << std::endl;
    std::cout << "Saved water to " << argv[2] << std::endl;
    std::cout << "Saved shore to " << argv[3] << std::endl;

    return 0;
}