// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#ifndef DYNAMIC_INTERFACE__PUBLISHER_HPP_
#define DYNAMIC_INTERFACE__PUBLISHER_HPP_

#include <memory>
#include <utility>
#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dynamic_interface
{

class Publisher
{
public:
  virtual ~Publisher() = default;

  Publisher() = delete;
  Publisher(const Publisher &) = delete;
  Publisher & operator=(const Publisher &) = delete;
  Publisher(Publisher &&) = delete;
  Publisher & operator=(Publisher &&) = delete;

public:
  Publisher(
    std::shared_ptr<rclcpp::Node> node,
    std::string name,
    std::string type,
    rclcpp::QoS qos = rclcpp::QoS(10));

public:
  void Publish(const nlohmann::json & data);

private:
  void RecursiveData(
    const nlohmann::json & data,
    void * message,
    const rosidl_typesupport_introspection_cpp::MessageMembers * members);

  void SetPrimitiveField(
    const nlohmann::json & value,
    void * field,
    const rosidl_typesupport_introspection_cpp::MessageMember & member);

  size_t GetPrimitiveSize(const rosidl_typesupport_introspection_cpp::MessageMember & member);

private:
  std::shared_ptr<rclcpp::Node> m_node;
  const std::string m_name;
  const std::string m_type;

private:
  std::shared_ptr<rclcpp::GenericPublisher> m_publisher;
  const rosidl_message_type_support_t * m_introspect;
  const rosidl_message_type_support_t * m_serialize;
  const rosidl_typesupport_introspection_cpp::MessageMembers * m_members;
};

}  // namespace dynamic_interface

#endif  // DYNAMIC_INTERFACE__PUBLISHER_HPP_
