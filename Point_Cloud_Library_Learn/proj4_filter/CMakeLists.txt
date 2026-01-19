# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(PointCloudDenoise)

# 设置 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 编译两个可执行文件
add_executable(step2_denoise src/step2_denoise.cpp)
add_executable(step3_segment_water src/step3_segment_water.cpp)

target_link_libraries(step2_denoise ${PCL_LIBRARIES})
target_link_libraries(step3_segment_water ${PCL_LIBRARIES})