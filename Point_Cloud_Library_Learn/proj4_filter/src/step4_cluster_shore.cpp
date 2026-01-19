// src/step4_cluster_shore.cpp
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cerr << "Usage: " << argv[0] 
                  << " <input_shore.pcd> <output_clusters.pcd>" << std::endl;
        return -1;
    }

    using PointT = pcl::PointXYZ;
    using PointCloudT = pcl::PointCloud<PointT>;

    // 1. 加载岸上点云
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud) == -1)
    {
        std::cerr << "Error: Could not read file " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "Loaded " << cloud->size() << " shore points." << std::endl;

    // 2. 创建 KD-Tree 用于邻域搜索
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    // 3. 欧氏聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.5);    // 点间距 < 0.5 米视为同一簇
    ec.setMinClusterSize(5);       // 至少 50 个点（防小噪点）
    ec.setMaxClusterSize(1000000);  // 上限很大，不限制大簇
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::cout << "Found " << cluster_indices.size() << " clusters." << std::endl;

    // 4. 合并所有有效簇（这里保留所有，你也可只取前 N 个大簇）
    PointCloudT::Ptr clustered_cloud(new PointCloudT);
    for (const auto& indices : cluster_indices)
    {
        for (int idx : indices.indices)
        {
            clustered_cloud->push_back((*cloud)[idx]);
        }
    }

    // 5. 保存结果
    // pcl::io::savePCDFileBinary(argv[2], *clustered_cloud);
    pcl::io::savePCDFileASCII(argv[2], *clustered_cloud); // ← ASCII 格式
    std::cout << "Saved " << clustered_cloud->size() 
              << " points in " << cluster_indices.size() << " clusters to " << argv[2] << std::endl;

    return 0;
}