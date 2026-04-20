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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

# Generate action interfaces FIRST (before executables)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrackPoles.action"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ============================================
# Executable: pole_detection_node (LaserScan version - like rc2026_head_finder)
# ============================================
add_executable(pole_detection_node
  src/pole_detection_node.cpp
)

# Add dependency on generated action interface
add_dependencies(pole_detection_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

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
  rclcpp_action
)

target_link_libraries(pole_detection_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# ============================================
# Executable: action_server (Standalone action server)
# ============================================
add_executable(action_server
  src/action_server.cpp
)

# Add dependency on generated action interface
add_dependencies(action_server ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_include_directories(action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
)

target_link_libraries(action_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp
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
)

ament_package()