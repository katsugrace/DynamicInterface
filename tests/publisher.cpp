// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <boost/test/unit_test.hpp>

#include <dynamic_interface/publisher.hpp>

namespace dynamic_interface::tests
{

BOOST_AUTO_TEST_CASE(ConstructorThrowsOnNullNode)
{
  BOOST_CHECK_THROW(Publisher(nullptr, "topic", "std_msgs/msg/String"), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(ConstructorThrowsOnEmptyTopic)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  BOOST_CHECK_THROW(Publisher(node, "", "std_msgs/msg/String"), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(ConstructorThrowsOnEmptyType)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  BOOST_CHECK_THROW(Publisher(node, "topic", ""), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(ConstructorFailsIfTypeDoesNotExist)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  BOOST_CHECK_THROW(Publisher(node, "topic", "nonexistent/Type"), std::runtime_error);
}

BOOST_AUTO_TEST_CASE(ConstructorSucceedsWithValidParameters)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  BOOST_CHECK_NO_THROW(Publisher(node, "topic", "std_msgs/msg/String", rclcpp::QoS(5)));
}

BOOST_AUTO_TEST_CASE(ConstructorSucceedsWithDefaultQoS)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  BOOST_CHECK_NO_THROW(Publisher(node, "topic", "std_msgs/msg/String"));
}

BOOST_AUTO_TEST_CASE(PublishWithEmptyJsonObject)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  Publisher publisher(node, "topic", "std_msgs/msg/String");
  boost::json::object data;

  BOOST_CHECK_NO_THROW(publisher.Publish(data));
}

BOOST_AUTO_TEST_CASE(PublishWithMissingFieldInJson)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  Publisher publisher(node, "topic", "std_msgs/msg/String");
  const auto data = boost::json::object{
    {"wrong_field", "Hello!"}
  };

  BOOST_CHECK_NO_THROW(publisher.Publish(data));
}

BOOST_AUTO_TEST_CASE(PublishWithCorrectJson)
{
  const auto node = std::make_shared<rclcpp::Node>("test_node");
  Publisher publisher(node, "topic", "std_msgs/msg/String");
  const auto data = boost::json::object{
    {"data", "Hello!"}
  };

  BOOST_CHECK_NO_THROW(publisher.Publish(data));
}

}  // namespace dynamic_interface::tests
