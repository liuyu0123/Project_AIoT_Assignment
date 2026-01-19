// src/step6_extract_shores_kmeans.cpp
#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

struct Point {
    float x, y;
};

int main(int argc, char** argv)
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <2d.pcd> <left.pcd> <right.pcd>\n";
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(argv[1], *cloud);

    // 按 X 排序
    std::vector<Point> points;
    for (const auto& p : cloud->points) {
        points.push_back({p.x, p.y});
    }
    std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
    });

    const float bin_size = 0.5f;
    pcl::PointCloud<pcl::PointXYZ> left_cloud, right_cloud;

    for (size_t i = 0; i < points.size(); ) {
        float x_start = points[i].x;
        std::vector<float> y_vals;
        size_t j = i;
        while (j < points.size() && points[j].x < x_start + bin_size) {
            y_vals.push_back(points[j].y);
            ++j;
        }

        if (y_vals.size() < 6) continue; // 至少6个点才做聚类

        // === K-means (k=2) ===
        std::vector<float> sorted_y = y_vals;
        std::sort(sorted_y.begin(), sorted_y.end());

        // 初始化中心：取前1/3和后1/3的中位数
        float c1 = sorted_y[sorted_y.size()/3];
        float c2 = sorted_y[2*sorted_y.size()/3];

        // 迭代分配
        std::vector<int> labels(y_vals.size());
        bool changed = true;
        int iter = 0;
        while (changed && iter < 10) {
            changed = false;
            for (size_t k = 0; k < y_vals.size(); ++k) {
                float d1 = std::abs(y_vals[k] - c1);
                float d2 = std::abs(y_vals[k] - c2);
                if (d1 < d2) {
                    if (labels[k] != 0) changed = true;
                    labels[k] = 0;
                } else {
                    if (labels[k] != 1) changed = true;
                    labels[k] = 1;
                }
            }

            // 更新中心
            float sum0 = 0, cnt0 = 0;
            float sum1 = 0, cnt1 = 0;
            for (size_t k = 0; k < y_vals.size(); ++k) {
                if (labels[k] == 0) { sum0 += y_vals[k]; ++cnt0; }
                else { sum1 += y_vals[k]; ++cnt1; }
            }
            if (cnt0 > 0) c1 = sum0 / cnt0;
            if (cnt1 > 0) c2 = sum1 / cnt1;
            ++iter;
        }

        // 找出两个簇的代表值（中位数）
        std::vector<float> cluster0, cluster1;
        for (size_t k = 0; k < y_vals.size(); ++k) {
            if (labels[k] == 0) cluster0.push_back(y_vals[k]);
            else cluster1.push_back(y_vals[k]);
        }

        if (cluster0.empty() || cluster1.empty()) continue;

        auto median = [](const std::vector<float>& v) -> float {
            return v[v.size()/2];
        };

        float cand_left_y = median(cluster0);
        float cand_right_y = median(cluster1);

        // 强制 left < right
        if (cand_left_y > cand_right_y) std::swap(cand_left_y, cand_right_y);

        float x_center = x_start + bin_size / 2.0f;
        left_cloud.points.emplace_back(x_center, cand_left_y, 0.0f);
        right_cloud.points.emplace_back(x_center, cand_right_y, 0.0f);
    }

    auto finalize = [](pcl::PointCloud<pcl::PointXYZ>& c) {
        c.width = c.points.size(); c.height = 1; c.is_dense = false;
    };
    finalize(left_cloud);
    finalize(right_cloud);

    pcl::io::savePCDFileASCII(argv[2], left_cloud);
    pcl::io::savePCDFileASCII(argv[3], right_cloud);

    std::cout << "K-means: Left=" << left_cloud.size() << ", Right=" << right_cloud.size() << "\n";
    return 0;
}