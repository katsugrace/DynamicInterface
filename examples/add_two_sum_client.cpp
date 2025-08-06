// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <memory>

#include <boost/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dynamic_interface/service_client.hpp>

int main(int argc, char * argv[])
{
  if (!rclcpp::ok()) {
    rclcpp::init(argc, argv);
  }

  const auto node = rclcpp::Node::make_shared("add_two_sum_client");

  std::shared_ptr<dynamic_interface::Client> client;
  try {
    client = std::make_shared<dynamic_interface::Client>(
      node, "/add_two_ints", "example_interfaces/srv/AddTwoInts");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create client: %s", e.what());
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    return EXIT_FAILURE;
  }

  std::atomic<bool> responseReceived(false);
  std::thread serviceThread([logger = node->get_logger(), &client, &responseReceived]() {
    const boost::json::object request = {
      {"a", 14},
      {"b", 2}
    };

    try {
      boost::json::value result = client->Call(request, std::chrono::seconds(5));
      std::cout << boost::json::serialize(result) << '\n';
      RCLCPP_INFO(logger, "Response: %s", boost::json::serialize(result).c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger, "Service call failed: %s", e.what());
    }

    responseReceived = true;
  });

  while (rclcpp::ok() && !responseReceived) {
    rclcpp::spin_some(node);
  }

  if (serviceThread.joinable()) {
    serviceThread.join();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return EXIT_SUCCESS;
}
