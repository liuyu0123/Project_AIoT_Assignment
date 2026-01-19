# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(PointCloudDenoise)

# 设置 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 PCL 库（必需）
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters segmentation)

# 包含 PCL 头文件目录
include_directories(${PCL_INCLUDE_DIRS})

# 链接 PCL 库所需定义和链接库
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(step2_denoise src/step2_denoise.cpp)
add_executable(step3_segment_water src/step3_segment_water.cpp)
add_executable(step4_cluster_shore src/step4_cluster_shore.cpp)
add_executable(step5_project_2d src/step5_project_2d.cpp)
add_executable(step6_extract_centerlines src/step6_extract_centerlines.cpp)
add_executable(step6_extract_centerlines2 src/step6_extract_centerlines_qwen.cpp)
add_executable(step7_interpolate_shores src/step7_interpolate_shores.cpp)

# 链接 PCL 库
target_link_libraries(step2_denoise ${PCL_LIBRARIES})
target_link_libraries(step3_segment_water ${PCL_LIBRARIES})
target_link_libraries(step4_cluster_shore ${PCL_LIBRARIES})
target_link_libraries(step5_project_2d ${PCL_LIBRARIES})
target_link_libraries(step6_extract_centerlines ${PCL_LIBRARIES})
target_link_libraries(step6_extract_centerlines2 ${PCL_LIBRARIES})
target_link_libraries(step7_interpolate_shores ${PCL_LIBRARIES})