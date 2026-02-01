#include "freespace_lidar_node/river_mapper.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<freespace_mapping::RiverMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}