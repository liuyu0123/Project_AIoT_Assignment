// src/step4_cluster_shore_labeled.cpp
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
                  << " <input_shore.pcd> <output_labeled.pcd>" << std::endl;
        return -1;
    }

    // 输入：普通点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        std::cerr << "Error: Could not read file " << argv[1] << std::endl;
        return -1;
    }

    // 创建 KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // 聚类
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2.0f);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(1000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::cout << "Found " << cluster_indices.size() << " clusters." << std::endl;

    // === 关键：创建带 label 的点云 ===
    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    labeled_cloud->header = cloud->header; // 保留时间戳等
    labeled_cloud->points.resize(cloud->size());
    labeled_cloud->width = cloud->size();
    labeled_cloud->height = 1;
    labeled_cloud->is_dense = false;

    // 初始化所有点 label = 0（或 -1，但 PCD 不支持负 label）
    for (auto& pt : labeled_cloud->points) {
        pt.label = 0; // 默认类别
    }

    // 给每个簇分配唯一 label（从 1 开始）
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        uint32_t label = static_cast<uint32_t>(i + 1); // label 从 1 开始
        for (int idx : cluster_indices[i].indices)
        {
            labeled_cloud->points[idx].x = (*cloud)[idx].x;
            labeled_cloud->points[idx].y = (*cloud)[idx].y;
            labeled_cloud->points[idx].z = (*cloud)[idx].z;
            labeled_cloud->points[idx].label = label;
        }
    }

    // 保存为 PCD（ASCII 更兼容）
    pcl::io::savePCDFileASCII(argv[2], *labeled_cloud);
    std::cout << "Saved labeled point cloud to " << argv[2] << std::endl;

    return 0;
}