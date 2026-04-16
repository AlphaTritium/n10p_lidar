cmake_minimum_required(VERSION 3.5)
project(test_simple)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()