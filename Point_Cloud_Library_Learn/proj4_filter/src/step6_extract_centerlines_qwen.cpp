#include <iostream>
#include <vector>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

struct Point2 { float x, y; };

// ---------- 1. 全局直方图谷值 ----------
float computeValleyCenter(const pcl::PointCloud<pcl::PointXYZ>& cloud,
                          int hist_bins = 200)
{
    if (cloud.empty()) return 0.0f;
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(cloud, min_pt, max_pt);
    float y_min = min_pt.y, y_max = max_pt.y;
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

// ---------- 2. 逐 bin k-means(k=2) ----------
void kmeans2(const std::vector<float>& y,
             float& y_left,
             float& y_right)
{
    size_t n = y.size();
    if (n < 6) { y_left = y_right = 0; return; }
    float c1 = y[n/3], c2 = y[2*n/3];
    std::vector<int> label(n, 0);
    bool changed = true; int it = 0;
    while (changed && it < 10) {
        changed = false;
        for (size_t i = 0; i < n; ++i) {
            int old = label[i];
            label[i] = (std::abs(y[i] - c1) < std::abs(y[i] - c2)) ? 0 : 1;
            if (old != label[i]) changed = true;
        }
        float s0 = 0, s1 = 0; size_t cnt0 = 0, cnt1 = 0;
        for (size_t i = 0; i < n; ++i) {
            if (label[i] == 0) { s0 += y[i]; ++cnt0; }
            else               { s1 += y[i]; ++cnt1; }
        }
        if (cnt0) c1 = s0 / cnt0;
        if (cnt1) c2 = s1 / cnt1;
        ++it;
    }
    // 全局校准：谁 y 小谁就是左岸
    if (c1 < c2) { y_left = c1; y_right = c2; }
    else         { y_left = c2; y_right = c1; }
}

// ---------- 3. 主函数 ----------
int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <input_2d.pcd> <left.pcd> <right.pcd> [bin_m] [minPts]\n";
        return -1;
    }
    const float bin_m   = (argc > 4) ? std::stof(argv[4]) : 0.5f;
    const size_t minPts = (argc > 5) ? std::stoi(argv[5]) : 6;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
        std::cerr << "Error reading: " << argv[1] << std::endl;
        return -1;
    }
    std::cout << "[INFO] Loaded " << cloud->size() << " 2-D shore points.\n";

    // ① 全局谷值定半区
    float y_center = computeValleyCenter(*cloud);
    std::cout << "[INFO] Global valley center: " << y_center << std::endl;

    // ② 预排序
    std::vector<Point2> pts;
    pts.reserve(cloud->size());
    for (const auto& p : cloud->points) pts.push_back({p.x, p.y});
    std::sort(pts.begin(), pts.end(), [](const Point2& a, const Point2& b) {
        return a.x < b.x;
    });

    // ③ 滑动窗口分 bin + k-means 精修
    pcl::PointCloud<pcl::PointXYZ> left_cloud, right_cloud;
    float x_start = pts.front().x;
    size_t left_idx = 0;
    float last_left = 0, last_right = 0;
    bool valid_last = false;

    while (left_idx < pts.size()) {
        float x_end = x_start + bin_m;
        size_t right_idx = left_idx;
        while (right_idx < pts.size() && pts[right_idx].x < x_end) ++right_idx;
        size_t cnt = right_idx - left_idx;
        if (cnt >= minPts) {
            std::vector<float> y_vals(cnt);
            for (size_t i = 0; i < cnt; ++i) y_vals[i] = pts[left_idx + i].y;
            std::sort(y_vals.begin(), y_vals.end());
            float y_left, y_right;
            kmeans2(y_vals, y_left, y_right);
            float x_mid = (x_start + x_end) / 2.0f;
            // ④ 用全局中心线再校验一次：k-means 结果必须落在对应半区
            if (y_left  < y_center && y_right > y_center) {
                left_cloud.emplace_back(x_mid, y_left,  0.0f);
                right_cloud.emplace_back(x_mid, y_right, 0.0f);
                last_left = y_left; last_right = y_right; valid_last = true;
            } else {
                // 落在同一侧 → 退回到谷值版极值
                float yl = *std::max_element(y_vals.begin(), y_vals.end());
                float yr = *std::min_element(y_vals.begin(), y_vals.end());
                if (yl < y_center && yr > y_center) {
                    left_cloud.emplace_back(x_mid, yl, 0.0f);
                    right_cloud.emplace_back(x_mid, yr, 0.0f);
                    last_left = yl; last_right = yr; valid_last = true;
                } else {
                    // 仍异常 → 外插
                    if (valid_last) {
                        left_cloud.emplace_back(x_mid, last_left, 0.0f);
                        right_cloud.emplace_back(x_mid, last_right, 0.0f);
                    }
                }
            }
        } else if (valid_last) {
            // 断线补偿
            float x_mid = (x_start + x_end) / 2.0f;
            left_cloud.emplace_back(x_mid, last_left, 0.0f);
            right_cloud.emplace_back(x_mid, last_right, 0.0f);
        }
        left_idx = right_idx;
        x_start  = x_end;
    }

    auto finalize = [](pcl::PointCloud<pcl::PointXYZ>& c) {
        c.width = static_cast<uint32_t>(c.size());
        c.height = 1;
        c.is_dense = false;
    };
    finalize(left_cloud);
    finalize(right_cloud);
    pcl::io::savePCDFileASCII(argv[2], left_cloud);
    pcl::io::savePCDFileASCII(argv[3], right_cloud);
    std::cout << "[INFO] Hybrid: Left=" << left_cloud.size()
              << "  Right=" << right_cloud.size() << std::endl;
    return 0;
}