cmake_minimum_required(VERSION 3.8)
project(pole_detection)

# Suppress PCL_ROOT warning
cmake_policy(SET CMP0074 NEW)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies - ORDER MATTERS!
find_package(Boost REQUIRED thread)
find_package(diagnostic_updater REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation kdtree search surface)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rc2026_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${PCL_COMMON_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${PCL_LIBRARY_DIRS}
)

# Add definitions for PCL
add_definitions(${PCL_DEFINITIONS})

# ============================================
# Executable: pole_detection_node (Main orchestrator)
# ============================================
add_executable(pole_detection_node
  src/pole_detection_node.cpp
  # src/preprocessor.cpp
  src/clusterer.cpp
  src/validator.cpp
  src/tracker.cpp
  src/pattern_matcher.cpp
)

target_include_directories(pole_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pole_detection_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  lslidar_msgs
  pcl_conversions
)

target_link_libraries(pole_detection_node
  ${PCL_LIBRARIES}
)

# ============================================
# Executable: action_server (Standalone action server)
# ============================================
add_executable(action_server
  src/action_server.cpp
)

target_include_directories(action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  rc2026_interfaces
  lslidar_msgs
  geometry_msgs
  std_msgs
)

# ============================================
# Install targets
# ============================================
install(TARGETS
  pole_detection_node
  action_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ============================================
# Testing
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export include directories
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  lslidar_msgs
  pcl_conversions
  rclcpp_action
  rc2026_interfaces
)

ament_package()