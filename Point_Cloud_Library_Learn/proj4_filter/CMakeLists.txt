# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(PointCloudDenoise)

# 设置 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 PCL 库（必需）
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters)

# 包含 PCL 头文件目录
include_directories(${PCL_INCLUDE_DIRS})

# 链接 PCL 库所需定义和链接库
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(step2_denoise src/step2_denoise.cpp)
add_executable(step3_segment_water src/step3_segment_water.cpp)

# 链接 PCL 库
target_link_libraries(step2_denoise ${PCL_LIBRARIES})
target_link_libraries(step3_segment_water ${PCL_LIBRARIES})