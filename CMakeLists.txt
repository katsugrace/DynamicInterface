cmake_minimum_required(VERSION 3.10)
project(ros2_dynamic_pub)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_dynamic_typesupport REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

add_executable(dynamic_pub
  src/main.cpp
  src/publisher.cpp
)

ament_target_dependencies(dynamic_pub
  rclcpp
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
  rosidl_dynamic_typesupport
)

target_link_libraries(dynamic_pub
  nlohmann_json::nlohmann_json
)

install(TARGETS dynamic_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
