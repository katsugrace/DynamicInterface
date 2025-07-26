cmake_minimum_required(VERSION 3.10)
project(dynamic_interface_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(dynamic_interface REQUIRED)

add_executable(float_publisher
  float_publisher.cpp
)

ament_target_dependencies(float_publisher
  rclcpp
  std_msgs
  nlohmann_json
  dynamic_interface
)

install(TARGETS float_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
