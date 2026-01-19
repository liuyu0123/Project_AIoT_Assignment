// src/utils_interpolate_shore.cpp
#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct Point { float x, y; };
bool operator<(const Point& a, const Point& b) { return a.x < b.x; }

std::vector<Point> loadAndSort(const std::string& file) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file, *cloud);
    std::vector<Point> pts;
    for (auto& p : cloud->points) pts.push_back({p.x, p.y});
    std::sort(pts.begin(), pts.end());
    return pts;
}

// 线性插值（简单有效）
std::vector<Point> resample(const std::vector<Point>& pts, float dx = 0.2f) {
    if (pts.size() < 2) return pts;
    std::vector<Point> out;
    float x_start = pts.front().x;
    float x_end = pts.back().x;
    for (float x = x_start; x <= x_end; x += dx) {
        // 找左右邻居
        auto it = std::lower_bound(pts.begin(), pts.end(), Point{x, 0});
        if (it == pts.begin()) {
            out.push_back(*it);
        } else if (it == pts.end()) {
            out.push_back(pts.back());
        } else {
            const Point& p1 = *(it - 1);
            const Point& p2 = *it;
            float t = (x - p1.x) / (p2.x - p1.x);
            float y = p1.y + t * (p2.y - p1.y);
            out.push_back({x, y});
        }
    }
    return out;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <left.pcd> <right.pcd> <output_dir>\n";
        return -1;
    }

    auto left_raw = loadAndSort(argv[1]);
    auto right_raw = loadAndSort(argv[2]);

    if (left_raw.empty() || right_raw.empty()) return -1;

    // 统一 X 范围
    float x_min = std::max(left_raw.front().x, right_raw.front().x);
    float x_max = std::min(left_raw.back().x, right_raw.back().x);
    if (x_min >= x_max) return -1;

    // 截断到公共区间
    auto crop = [&](std::vector<Point>& pts) {
        pts.erase(std::remove_if(pts.begin(), pts.end(), [&](const Point& p) {
            return p.x < x_min || p.x > x_max;
        }), pts.end());
    };
    crop(left_raw);
    crop(right_raw);

    // 重采样
    auto left_interp = resample(left_raw, 0.2f);
    auto right_interp = resample(right_raw, 0.2f);

    // 保存为 PCD（用于调试）
    auto savePCD = [](const std::vector<Point>& pts, const std::string& name) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (auto& p : pts) cloud.points.emplace_back(p.x, p.y, 0);
        cloud.width = cloud.points.size(); cloud.height = 1; cloud.is_dense = false;
        pcl::io::savePCDFileASCII(name, cloud);
    };

    savePCD(left_interp, std::string(argv[3]) + "/left_interp.pcd");
    savePCD(right_interp, std::string(argv[3]) + "/right_interp.pcd");

    std::cout << "Interpolated to " << left_interp.size() << " points.\n";
    return 0;
}