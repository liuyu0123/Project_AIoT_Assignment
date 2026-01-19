#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

// ------------ 直方图谷值求航道中心 --------------
float computeValleyCenter(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                          int hist_bins = 200)
{
    if (cloud.empty()) return 0.0f;
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    float y_min = min_pt.y;
    float y_max = max_pt.y;

    const float bin_w = (y_max - y_min) / hist_bins;
    std::vector<int> hist(hist_bins, 0);
    for (const auto& p : cloud.points) {
        int idx = static_cast<int>((p.y - y_min) / bin_w);
        idx = std::max(0, std::min(idx, hist_bins - 1));
        ++hist[idx];
    }
    int left_peak  = std::max_element(hist.begin(), hist.begin() + hist_bins / 2) - hist.begin();
    int right_peak = std::max_element(hist.begin() + hist_bins / 2, hist.end()) - hist.begin();
    if (left_peak >= right_peak) right_peak = hist_bins - 1;
    int valley = std::min_element(hist.begin() + left_peak, hist.begin() + right_peak) - hist.begin();
    return y_min + valley * bin_w;
}
// ----------------------------------------------

int main(int argc, char** argv)
{
    // 手工解析：要么 4 参数，要么 6 参数（带 --axis x）
    if (argc != 4 && argc != 6) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_2d.pcd> <left.pcd> <right.pcd>  [--axis x|y|z]\n"
                  << "Note: --axis must be the 5th and 6th argument if present.\n";
        return -1;
    }
    std::string axis = "x";   // 默认
    if (argc == 6) {
        if (std::string(argv[4]) != "--axis") {
            std::cerr << "Error: expected --axis as 5th argument.\n";
            return -1;
        }
        axis = argv[5];
        if (axis != "x" && axis != "y" && axis != "z") {
            std::cerr << "Error: axis must be x, y or z.\n";
            return -1;
        }
    }
    // 目前只实现 x 轴
    if (axis != "x") {
        std::cerr << "[ERROR] Only --axis x is implemented now.\n";
        return -1;
    }

    const std::string proj_file = argv[1];
    const std::string left_file = argv[2];
    const std::string right_file = argv[3];

    // 加载
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(proj_file, *cloud) == -1) {
        std::cerr << "Error reading: " << proj_file << std::endl;
        return -1;
    }
    std::cout << "[INFO] Loaded " << cloud->size() << " 2-D shore points.\n";

    float y_center = computeValleyCenter(*cloud);
    std::cout << "[INFO] Valley-based Y center: " << y_center << std::endl;

    // 以下与前一版完全相同 ********************
    const float bin_size = 0.5f;
    float x_min = 1e9, x_max = -1e9;
    for (const auto& p : cloud->points) {
        x_min = std::min(x_min, p.x);
        x_max = std::max(x_max, p.x);
    }
    int num_bins = static_cast<int>((x_max - x_min) / bin_size) + 1;
    std::vector<std::vector<float>> left_candidates(num_bins);
    std::vector<std::vector<float>> right_candidates(num_bins);

    for (const auto& p : cloud->points) {
        int bin_idx = static_cast<int>((p.x - x_min) / bin_size);
        bin_idx = std::max(0, std::min(bin_idx, num_bins - 1));
        if (p.y < y_center) left_candidates[bin_idx].push_back(p.y);
        else if (p.y > y_center) right_candidates[bin_idx].push_back(p.y);
    }

    pcl::PointCloud<pcl::PointXYZ> left_cloud, right_cloud;
    for (int i = 0; i < num_bins; ++i) {
        if (!left_candidates[i].empty()) {
            float y_left = *std::max_element(left_candidates[i].begin(), left_candidates[i].end());
            left_cloud.emplace_back(x_min + (i + 0.5f) * bin_size, y_left, 0.0f);
        }
        if (!right_candidates[i].empty()) {
            float y_right = *std::min_element(right_candidates[i].begin(), right_candidates[i].end());
            right_cloud.emplace_back(x_min + (i + 0.5f) * bin_size, y_right, 0.0f);
        }
    }

    auto finalize = [](pcl::PointCloud<pcl::PointXYZ>& c) {
        c.width = static_cast<uint32_t>(c.size());
        c.height = 1;
        c.is_dense = false;
    };
    finalize(left_cloud);
    finalize(right_cloud);

    pcl::io::savePCDFileASCII(left_file, left_cloud);
    pcl::io::savePCDFileASCII(right_file, right_cloud);
    std::cout << "[INFO] Left: " << left_cloud.size()
              << ", Right: " << right_cloud.size() << " points.\n";
    return 0;
}