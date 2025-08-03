// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <memory>

#include <boost/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dynamic_interface/publisher.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("string_publisher");

  dynamic_interface::Publisher publisher(node, "/dynamic_topic", "std_msgs/msg/String");

  const auto data = boost::json::object{
    {"data", "Hello, ROS2!"}
  };

  rclcpp::Rate rate(1000);
  while (rclcpp::ok()) {
    publisher.Publish(data);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
