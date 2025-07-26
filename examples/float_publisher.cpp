#include <chrono>
#include <thread>
#include <memory>
#include <cstdlib>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <dynamic_interface/publisher.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("dynamic_json_pub");

  dynamic_interface::Publisher pub(node, "/dynamic_topic", "std_msgs/msg/Float64");

  nlohmann::json data = {
    {"data", 123.0}
  };

  rclcpp::Rate rate(1000);
  while (rclcpp::ok()) {
    pub.Publish(data);
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
