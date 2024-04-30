cmake_minimum_required(VERSION 3.8)
project(point_cloud_proc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY
    launch config
  DESTINATION
    share/${PROJECT_NAME}
)

include_directories(include/point_cloud_proc)

add_executable(filter_cloud_node src/filter_cloud.cpp)
target_include_directories(filter_cloud_node PUBLIC
  include
)
ament_target_dependencies(
  filter_cloud_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  # pcl_ros
)

install(
  TARGETS
    filter_cloud_node
  DESTINATION
    lib/${PROJECT_NAME}
  )
target_link_libraries(filter_cloud_node ${PCL_LIBRARIES})
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