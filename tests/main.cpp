// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#define BOOST_TEST_NO_MAIN 
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE DynamicInterfaceTest

#include <boost/test/unit_test.hpp>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  const int result = boost::unit_test::unit_test_main([]() {return true;}, argc, argv);

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return result;
}
