cmake_minimum_required(VERSION 3.8)
project(rc2026_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KFSPose.msg"
  "msg/KFSPoseArray.msg"
  "action/GripperControl.action"
  "action/StairAlignment.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime std_msgs action_msgs)
ament_package()