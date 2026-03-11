cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree search surface)

add_executable(lidar_app_node src/lidar_app_node.cpp)
ament_target_dependencies(lidar_app_node
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  lslidar_msgs
)

target_link_libraries(lidar_app_node
  ${PCL_LIBRARIES}
)

add_executable(lidar_pointcloud_processor src/lidar_pointcloud_processor.cpp)
ament_target_dependencies(lidar_pointcloud_processor
  rclcpp
  sensor_msgs
  pcl_conversions
  lslidar_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  std_msgs
  geometry_msgs
)

target_link_libraries(lidar_pointcloud_processor
  ${PCL_LIBRARIES}
)

install(TARGETS
  lidar_app_node
  lidar_pointcloud_processor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}/
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