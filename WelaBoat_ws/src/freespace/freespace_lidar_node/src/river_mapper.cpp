#include "freespace_lidar_node/river_mapper.hpp"

// ✅ 关键修复：包含 pcl_conversions 以支持 fromROSMsg
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <algorithm>
#include <vector>

using namespace freespace_mapping;

struct Point2D { float x, y; };
bool cmp_x(const Point2D& a, const Point2D& b) { return a.x < b.x; }

std::pair<std::vector<Point2D>, std::vector<Point2D>>
extract_shores(const pcl::PointCloud<pcl::PointXYZL>::Ptr& labeled_2d)
{
    std::vector<Point2D> all_pts;
    for (const auto& p : labeled_2d->points) {
        all_pts.push_back({p.x, p.y});
    }
    if (all_pts.empty()) return {{}, {}};

    auto y_min_max = std::minmax_element(all_pts.begin(), all_pts.end(),
        [](const Point2D& a, const Point2D& b) { return a.y < b.y; });
    float y_min = y_min_max.first->y;
    float y_max = y_min_max.second->y;

    const int hist_bins = 200;
    float bin_w = (y_max - y_min) / hist_bins;
    std::vector<int> hist(hist_bins, 0);
    for (auto& p : all_pts) {
        int idx = static_cast<int>((p.y - y_min) / bin_w);
        idx = std::clamp(idx, 0, hist_bins - 1);
        ++hist[idx];
    }

    int left_peak = std::max_element(hist.begin(), hist.begin() + hist_bins/2) - hist.begin();
    int right_peak = std::max_element(hist.begin() + hist_bins/2, hist.end()) - hist.begin();
    if (left_peak >= right_peak) right_peak = hist_bins - 1;
    int valley = std::min_element(hist.begin() + left_peak, hist.begin() + right_peak) - hist.begin();
    float y_center = y_min + valley * bin_w;

    std::sort(all_pts.begin(), all_pts.end(), cmp_x);

    float x_start = all_pts.front().x;
    float x_end = all_pts.back().x;
    float bin_size = 0.5f;
    int num_bins = static_cast<int>((x_end - x_start) / bin_size) + 1;

    std::vector<std::vector<float>> left_y(num_bins), right_y(num_bins);
    for (auto& p : all_pts) {
        int bin_idx = static_cast<int>((p.x - x_start) / bin_size);
        bin_idx = std::clamp(bin_idx, 0, num_bins - 1);
        if (p.y < y_center) left_y[bin_idx].push_back(p.y);
        else if (p.y > y_center) right_y[bin_idx].push_back(p.y);
    }

    std::vector<Point2D> left, right;
    for (int i = 0; i < num_bins; ++i) {
        if (!left_y[i].empty()) {
            float y_val = *std::max_element(left_y[i].begin(), left_y[i].end());
            left.push_back({x_start + (i + 0.5f) * bin_size, y_val});
        }
        if (!right_y[i].empty()) {
            float y_val = *std::min_element(right_y[i].begin(), right_y[i].end());
            right.push_back({x_start + (i + 0.5f) * bin_size, y_val});
        }
    }
    return {left, right};
}

std::vector<Point2D> resample_curve(const std::vector<Point2D>& pts, float dx)
{
    if (pts.size() < 2) return pts;
    float x0 = pts.front().x, x1 = pts.back().x;
    std::vector<Point2D> out;
    for (float x = x0; x <= x1; x += dx) {
        auto it = std::lower_bound(pts.begin(), pts.end(), Point2D{x,0}, cmp_x);
        if (it == pts.begin()) out.push_back(*it);
        else if (it == pts.end()) out.push_back(pts.back());
        else {
            const Point2D& p1 = *(it - 1);
            const Point2D& p2 = *it;
            float t = (x - p1.x) / (p2.x - p1.x);
            float y = p1.y + t * (p2.y - p1.y);
            out.push_back({x, y});
        }
    }
    return out;
}

RiverMapper::RiverMapper(const rclcpp::NodeOptions & options)
: Node("freespace_mapper", options)
{
    declare_parameter("output_dir", ".");
    declare_parameter("save_map_files", false);
    declare_parameter("map_frame_id", "map");

    output_dir_ = get_parameter("output_dir").as_string();
    save_map_files_ = get_parameter("save_map_files").as_bool();
    map_frame_id_ = get_parameter("map_frame_id").as_string();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/unilidar/cloud", 10,
        std::bind(&RiverMapper::pointcloud_callback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map", rclcpp::QoS(1).transient_local());
}

void RiverMapper::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // ✅ 正确使用 fromROSMsg（依赖 pcl_conversions）
    pcl::fromROSMsg(*msg, *cloud);

    // Step 1: Denoise
    pcl::PointCloud<pcl::PointXYZ>::Ptr denoised(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(30);
    sor.setStddevMulThresh(1.0);
    sor.filter(*denoised);

    // Step 2: Keep shore (z > 0.2)
    pcl::PointCloud<pcl::PointXYZ>::Ptr shore(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(denoised);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.2f, 100.0f);
    pass.filter(*shore);

    if (shore->empty()) {
        RCLCPP_WARN(this->get_logger(), "No shore points after filtering.");
        return;
    }

    // Step 3: Cluster
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(shore);
    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(2.0);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(1000000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(shore);
    ec.extract(indices);

    if (indices.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Less than 2 clusters found.");
        return;
    }

    // Label top 2 largest clusters
    std::sort(indices.begin(), indices.end(),
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size();
        });

    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled(new pcl::PointCloud<pcl::PointXYZL>);
    labeled->points.resize(shore->size());
    for (size_t i = 0; i < shore->size(); ++i) {
        labeled->points[i].x = (*shore)[i].x;
        labeled->points[i].y = (*shore)[i].y;
        labeled->points[i].z = 0; // project to 2D
        labeled->points[i].label = 0;
    }

    for (size_t i = 0; i < 2; ++i) {
        for (int idx : indices[i].indices) {
            labeled->points[idx].label = i + 1;
        }
    }

    // Step 4: Extract shores
    auto [left_raw, right_raw] = extract_shores(labeled);
    if (left_raw.empty() || right_raw.empty()) {
        RCLCPP_WARN(this->get_logger(), "Failed to extract left/right shores.");
        return;
    }

    // Align X range
    float x_min = std::max(left_raw.front().x, right_raw.front().x);
    float x_max = std::min(left_raw.back().x, right_raw.back().x);
    if (x_min >= x_max) return;

    auto crop = [&](std::vector<Point2D>& pts) {
        pts.erase(std::remove_if(pts.begin(), pts.end(),
            [&](const Point2D& p) { return p.x < x_min || p.x > x_max; }), pts.end());
    };
    crop(left_raw); crop(right_raw);
    if (left_raw.empty() || right_raw.empty()) return;

    // Step 5: Resample
    auto left_interp = resample_curve(left_raw, 0.2f);
    auto right_interp = resample_curve(right_raw, 0.2f);

    // Step 6: Build polygon
    std::vector<cv::Point2f> poly;
    for (auto& p : left_interp) poly.emplace_back(p.x, p.y);
    for (auto it = right_interp.rbegin(); it != right_interp.rend(); ++it)
        poly.emplace_back(it->x, it->y);

    if (poly.size() < 3) return;

    // Compute bounds
    float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    for (auto& p : poly) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    float resolution = 0.1f;
    auto grid_msg = create_occupancy_grid(poly, min_x, max_x, min_y, max_y, resolution);
    map_pub_->publish(grid_msg);
    RCLCPP_INFO(this->get_logger(), "Published /map");

    // Optional: save files
    if (save_map_files_) {
        int width = grid_msg.info.width;
        int height = grid_msg.info.height;
        cv::Mat img(height, width, CV_8UC1, cv::Scalar(205)); // default unknown

        for (size_t i = 0; i < grid_msg.data.size(); ++i) {
            int8_t val = grid_msg.data[i];
            if (val == 100) img.data[i] = 0;     // occupied -> black
            else if (val == 0) img.data[i] = 254; // free -> white-ish
        }

        std::string pgm_file = output_dir_ + "/river_map.pgm";
        std::string yaml_file = output_dir_ + "/river_map.yaml";
        cv::imwrite(pgm_file, img);

        std::ofstream yaml(yaml_file);
        yaml << "image: river_map.pgm\n";
        yaml << "resolution: " << resolution << "\n";
        yaml << "origin: [" << min_x << ", " << min_y << ", 0.0]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.65\n";
        yaml << "free_thresh: 0.196\n";
        yaml.close();

        RCLCPP_INFO(this->get_logger(), "Saved map files to %s", output_dir_.c_str());
    }
}

nav_msgs::msg::OccupancyGrid RiverMapper::create_occupancy_grid(
    const std::vector<cv::Point2f>& poly,
    float min_x, float max_x, float min_y, float max_y,
    float resolution)
{
    int width = static_cast<int>((max_x - min_x) / resolution) + 1;
    int height = static_cast<int>((max_y - min_y) / resolution) + 1;

    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC1);
    std::vector<cv::Point> poly_px;
    for (const auto& p : poly) {
        int px = static_cast<int>((p.x - min_x) / resolution);
        int py = height - 1 - static_cast<int>((p.y - min_y) / resolution);
        poly_px.emplace_back(px, py);
    }
    cv::fillPoly(img, std::vector<std::vector<cv::Point>>{poly_px}, cv::Scalar(255));

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = map_frame_id_;
    grid.header.stamp = this->now();
    grid.info.resolution = resolution;
    grid.info.width = width;
    grid.info.height = height;
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(width * height, -1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint8_t val = img.at<uint8_t>(y, x);
            int idx = y * width + x;
            grid.data[idx] = (val > 128) ? 100 : 0;
        }
    }

    return grid;
}