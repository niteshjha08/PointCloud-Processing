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
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

add_executable(preprocess_cloud_node src/preprocess_cloud.cpp)
target_include_directories(preprocess_cloud_node PUBLIC
  include
)

add_executable(plane_segmentation_node src/plane_segmentation.cpp)
target_include_directories(plane_segmentation_node PUBLIC
  include
)

add_executable(clustering_node src/clustering.cpp)
target_include_directories(clustering_node PUBLIC
  include
)

ament_target_dependencies(
  preprocess_cloud_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  # pcl_ros
)

ament_target_dependencies(
  plane_segmentation_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_geometry_msgs
  # pcl_ros
)

ament_target_dependencies(
  clustering_node
  rclcpp
  sensor_msgs
  pcl_conversions
  PCL
  tf2_ros
  tf2_geometry_msgs
  # pcl_ros
)

install(
  TARGETS
  preprocess_cloud_node plane_segmentation_node clustering_node
  DESTINATION
    lib/${PROJECT_NAME}
  )
target_link_libraries(preprocess_cloud_node ${PCL_LIBRARIES})
target_link_libraries(plane_segmentation_node ${PCL_LIBRARIES})
target_link_libraries(clustering_node ${PCL_LIBRARIES})
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArrayPointCloud.msg"
  DEPENDENCIES std_msgs sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(clustering_node "${cpp_typesupport_target}") 

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
