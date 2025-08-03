// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <memory>

#include <boost/json.hpp>

#include <rclcpp/rclcpp.hpp>

#include <dynamic_interface/subscriber.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("string_subscriber");

  std::shared_ptr<dynamic_interface::Subscriber> subscriber;
  try {
    subscriber = std::make_shared<dynamic_interface::Subscriber>(
      node,
      "/dynamic_topic",
      "std_msgs/msg/String",
      [](const boost::json::value & data) {
        RCLCPP_INFO(
          rclcpp::get_logger("StringSubscriber"),
          "Message received: %s", boost::json::serialize(data).c_str());});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), e.what());
    return EXIT_FAILURE;
  }

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return EXIT_SUCCESS;
}
