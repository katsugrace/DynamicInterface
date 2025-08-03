// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#ifndef DYNAMIC_INTERFACE__SUBSCRIBER_HPP_
#define DYNAMIC_INTERFACE__SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include <boost/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dynamic_interface
{

namespace json = boost::json;

class Subscriber
{
public:
  using CallbackFunc = std::function<void (const json::value &)>;

public:
  virtual ~Subscriber() = default;

  Subscriber() = delete;
  Subscriber(const Subscriber &) = delete;
  Subscriber & operator=(const Subscriber &) = delete;
  Subscriber(Subscriber &&) = delete;
  Subscriber & operator=(Subscriber &&) = delete;

public:
  Subscriber(
    std::shared_ptr<rclcpp::Node> node,
    std::string name,
    std::string type,
    CallbackFunc callback,
    rclcpp::QoS qos = rclcpp::QoS(10));

private:
  void OnMessage(const rclcpp::SerializedMessage & data);
  void ToJson(
    void * message,
    const rosidl_typesupport_introspection_cpp::MessageMembers * members,
    json::object & outData);

  json::value ExtractPrimitive(void * field, const rosidl_typesupport_introspection_cpp::MessageMember & member);
  size_t GetPrimitiveSize(const rosidl_typesupport_introspection_cpp::MessageMember & member);  

private:
  std::shared_ptr<rclcpp::Node> m_node;
  std::string m_name;
  std::string m_type;
  CallbackFunc m_callback;

private:
  std::shared_ptr<rclcpp::GenericSubscription> m_subscription;
  const rosidl_message_type_support_t * m_introspect;
  const rosidl_message_type_support_t * m_serialize;
  const rosidl_typesupport_introspection_cpp::MessageMembers * m_members;
};

}  // namespace dynamic_interface

#endif  // DYNAMIC_INTERFACE__SUBSCRIBER_HPP_
