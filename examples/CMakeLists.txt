cmake_minimum_required(VERSION 3.10)
project(dynamic_interface_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamic_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS json)

set(PUBLISHERS
  float_publisher
  string_publisher
  bool_publisher

  float_subscriber
  string_subscriber
  bool_subscriber

  add_two_sum_client
)

foreach(pub ${PUBLISHERS})
  add_executable(${pub} ${pub}.cpp)

  target_include_directories(${pub} PRIVATE
    ${Boost_INCLUDE_DIRS}
  )

  ament_target_dependencies(${pub}
    rclcpp
    dynamic_interface
  )

  target_link_libraries(${pub}
    ${Boost_LIBRARIES}
  )

  install(TARGETS ${pub}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

ament_package()
