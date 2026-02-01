cmake_minimum_required(VERSION 3.8)
project(freespace_lidar_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED)
find_package(Boost REQUIRED COMPONENTS date_time)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
add_executable(river_mapper_node
  src/main.cpp
  src/river_mapper.cpp
)
target_link_libraries(river_mapper_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)
ament_target_dependencies(river_mapper_node 
  rclcpp 
  sensor_msgs 
  nav_msgs
  pcl_ros 
  pcl_conversions
)
install(TARGETS river_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
