// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <dynamic_interface/publisher.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dynamic_json_pub");

    dynamic_interface::Publisher pub(node, "/dynamic_topic", "std_msgs/msg/Float64");

    nlohmann::json j = {
        {"data", 123}
    };

    while (rclcpp::ok()) {
        pub.Publish(j);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    return 0;
}
