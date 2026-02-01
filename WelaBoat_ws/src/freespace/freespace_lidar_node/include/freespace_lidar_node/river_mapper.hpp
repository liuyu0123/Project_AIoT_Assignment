#ifndef FREESPACE_LIDAR_NODE_RIVER_MAPPER_HPP_
#define FREESPACE_LIDAR_NODE_RIVER_MAPPER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

// ✅ 关键修复：包含 OpenCV 头文件，使 cv::Point2f 可用
#include <opencv2/opencv.hpp>

namespace freespace_mapping {

class RiverMapper : public rclcpp::Node
{
public:
  explicit RiverMapper(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  nav_msgs::msg::OccupancyGrid create_occupancy_grid(
      const std::vector<cv::Point2f>& polygon,
      float min_x, float max_x, float min_y, float max_y,
      float resolution);

  // Parameters
  std::string output_dir_;
  bool save_map_files_;
  std::string map_frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

}  // namespace freespace_mapping

#endif  // FREESPACE_LIDAR_NODE_RIVER_MAPPER_HPP_