// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#ifndef DYNAMIC_INTERFACE__SERVICE_CLIENT_HPP_
#define DYNAMIC_INTERFACE__SERVICE_CLIENT_HPP_

#include <memory>
#include <string>
#include <chrono>

#include <boost/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

namespace dynamic_interface
{

namespace json = boost::json;

class Client
{
public:
  virtual ~Client() = default;

  Client() = delete;
  Client(const Client &) = delete;
  Client & operator=(const Client &) = delete;
  Client(Client &&) = delete;
  Client & operator=(Client &&) = delete;

public:
  Client(
    std::shared_ptr<rclcpp::Node> node,
    std::string name,
    std::string type);

public:
  json::value Call(
    const json::value & data,
    const std::chrono::seconds timeout = std::chrono::seconds(15));

private:
  void * FillRequest(const json::value & data);
  void RecursiveFill(
    const json::object & data,
    void * message,
    const rosidl_typesupport_introspection_cpp::MessageMembers * members);
  
  void FillPrimitive(
    const json::value & value,
    void * field,
    const rosidl_typesupport_introspection_cpp::MessageMember & member);

  json::value ExtractResponse(void * response);

private:
  std::shared_ptr<rclcpp::Node> m_node;
  std::string m_name;
  std::string m_type;

private:
  std::shared_ptr<rclcpp::GenericClient> m_client;
  const rosidl_service_type_support_t * m_introspect;
  const rosidl_service_type_support_t * m_serialize;
  const rosidl_typesupport_introspection_cpp::ServiceMembers * m_serviceMembers;
  const rosidl_typesupport_introspection_cpp::MessageMembers * m_requestMembers;
  const rosidl_typesupport_introspection_cpp::MessageMembers * m_responseMembers;
};

}  // namespace dynamic_interface

#endif  // DYNAMIC_INTERFACE__SERVICE_CLIENT_HPP_
