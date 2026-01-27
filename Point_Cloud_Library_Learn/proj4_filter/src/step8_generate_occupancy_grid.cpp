// src/step8_generate_occupancy_grid.cpp
#include <iostream>
#include <vector>
#include <algorithm>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

struct Point {
    float x, y;
};

// 加载 PCD 文件并返回排序后的点列表
std::vector<Point> loadPCD(const std::string& file) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(file, *cloud) == -1) {
        std::cerr << "ERROR: Cannot load PCD file: " << file << std::endl;
        return {};
    }

    std::vector<Point> pts;
    for (const auto& p : cloud->points) {
        pts.push_back({p.x, p.y});
    }

    // 按 X 坐标排序（确保沿航向）
    std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        return a.x < b.x;
    });

    return pts;
}

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] 
                  << " <left_interp.pcd> <right_interp.pcd> <map_prefix>\n";
        return -1;
    }

    auto left = loadPCD(argv[1]);
    auto right = loadPCD(argv[2]);

    if (left.empty() || right.empty()) {
        std::cerr << "ERROR: Empty shore point cloud!\n";
        return -1;
    }

    // 构建闭合多边形：左岸（前→后） + 右岸（后→前）
    std::vector<cv::Point2f> polygon;
    for (const auto& p : left) {
        polygon.push_back(cv::Point2f(p.x, p.y));
    }
    for (auto it = right.rbegin(); it != right.rend(); ++it) {
        polygon.push_back(cv::Point2f(it->x, it->y));
    }

    if (polygon.size() < 3) {
        std::cerr << "ERROR: Polygon too small!\n";
        return -1;
    }

    // 计算边界
    float min_x = 1e9f, max_x = -1e9f;
    float min_y = 1e9f, max_y = -1e9f;
    for (const auto& pt : polygon) {
        min_x = std::min(min_x, pt.x);
        max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y);
        max_y = std::max(max_y, pt.y);
    }

    const float resolution = 0.1f; // meters per pixel
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    // 创建地图图像（初始为黑色 = occupied）
    cv::Mat map_img = cv::Mat::zeros(height, width, CV_8UC1);

    // 转换多边形到像素坐标（注意：图像原点在左上，Y需翻转）
    std::vector<cv::Point> poly_px;
    for (const auto& pt : polygon) {
        int px = static_cast<int>((pt.x - min_x) / resolution);
        int py = height - 1 - static_cast<int>((pt.y - min_y) / resolution); // Y flip
        poly_px.push_back(cv::Point(px, py));
    }

    // 填充可行驶区域为白色（free space）
    cv::fillPoly(map_img, std::vector<std::vector<cv::Point>>{poly_px}, cv::Scalar(255));

    // 保存 .pgm 地图
    std::string pgm_file = std::string(argv[3]) + ".pgm";
    if (!cv::imwrite(pgm_file, map_img)) {
        std::cerr << "ERROR: Failed to write PGM file!\n";
        return -1;
    }

    // 保存 .yaml 配置文件（ROS 格式）
    std::string yaml_file = std::string(argv[3]) + ".yaml";
    std::ofstream yaml(yaml_file);
    if (!yaml.is_open()) {
        std::cerr << "ERROR: Cannot create YAML file!\n";
        return -1;
    }

    yaml << "image: " << pgm_file << "\n";
    yaml << "resolution: " << resolution << "\n";
    yaml << "origin: [" << min_x << ", " << min_y << ", 0.0]\n"; // ROS origin is bottom-left
    yaml << "negate: 0\n";
    yaml << "occupied_thresh: 0.65\n";
    yaml << "free_thresh: 0.196\n";
    yaml.close();

    std::cout << "[SUCCESS] Generated ROS map:\n"
              << "  - " << pgm_file << "\n"
              << "  - " << yaml_file << "\n";
    return 0;
}