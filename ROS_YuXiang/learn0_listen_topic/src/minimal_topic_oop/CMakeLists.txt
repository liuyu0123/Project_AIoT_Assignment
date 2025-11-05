cmake_minimum_required(VERSION 3.8)
project(minimal_topic_oop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

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


include_directories(include)

add_library(talker_node STATIC src/talker_node.cpp)
ament_target_dependencies(talker_node rclcpp std_msgs)

add_library(listener_node STATIC src/listener_node.cpp)
ament_target_dependencies(listener_node rclcpp std_msgs)

add_executable(talker src/main_talker.cpp)
target_link_libraries(talker talker_node)

add_executable(listener src/main_listener.cpp)
target_link_libraries(listener listener_node)

install(TARGETS talker listener
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()
