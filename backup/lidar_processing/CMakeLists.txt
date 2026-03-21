cmake_minimum_required(VERSION 3.8)
project(lidar_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree search)
find_package(lslidar_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add the executable
add_executable(lidar_processor src/lidar_processor.cpp)

target_include_directories(lidar_processor PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${lslidar_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
)

# Specify dependencies
ament_target_dependencies(lidar_processor
  rclcpp
  std_msgs
  sensor_msgs
  pcl_conversions
  lslidar_msgs
  visualization_msgs
)

target_link_libraries(lidar_processor
  ${PCL_LIBRARIES}
)

# Install the executable
install(TARGETS
  lidar_processor
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



