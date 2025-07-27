cmake_minimum_required(VERSION 3.10)
project(dynamic_interface_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamic_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS unit_test_framework json)

set(TEST_SOURCES
  main.cpp
  publisher.cpp
)

add_executable(${PROJECT_NAME} ${TEST_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
  ${Boost_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  dynamic_interface
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

add_test(NAME DynamicInterfaceTest COMMAND dynamic_interface_test)

ament_package()
