cmake_minimum_required(VERSION 3.10)
project(dynamic_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_dynamic_typesupport REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/publisher.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcutils
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  rosidl_dynamic_typesupport
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rcutils
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  rosidl_dynamic_typesupport
  nlohmann_json
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
