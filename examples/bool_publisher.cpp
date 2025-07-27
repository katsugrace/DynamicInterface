// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <thread>
#include <memory>
#include <cstdlib>

#include <boost/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dynamic_interface/publisher.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("bool_publsher");

  dynamic_interface::Publisher publisher(node, "/dynamic_topic", "std_msgs/msg/Bool");

  const auto data = boost::json::object{
    {"data", true}
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
