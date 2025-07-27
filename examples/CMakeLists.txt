cmake_minimum_required(VERSION 3.10)
project(dynamic_interface_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(dynamic_interface REQUIRED)

set(PUBLISHERS
  float_publisher
  string_publisher
  bool_publisher
)

foreach(pub ${PUBLISHERS})
  add_executable(${pub} ${pub}.cpp)
  ament_target_dependencies(${pub}
    rclcpp
    std_msgs
    nlohmann_json
    dynamic_interface
  )

  install(TARGETS ${pub}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
