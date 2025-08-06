// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <dynamic_interface/service_client.hpp>

namespace dynamic_interface
{

Client::Client(
  std::shared_ptr<rclcpp::Node> node,
  std::string name,
  std::string type)
: m_node(std::move(node))
, m_name(std::move(name))
, m_type(std::move(type))
{
  if (!m_node) {
    throw std::runtime_error("Node is null");
  }

  if (m_name.empty()) {
    throw std::runtime_error("Topic name is empty");
  }

  if (m_type.empty()) {
    throw std::runtime_error("Message type is empty");
  }

  m_client = m_node->create_generic_client(m_name, m_type);

  m_introspect = rclcpp::get_service_typesupport_handle(
    m_type, "rosidl_typesupport_introspection_cpp",
    *rclcpp::get_typesupport_library(m_type, "rosidl_typesupport_introspection_cpp"));

  if (!m_introspect) {
    throw std::runtime_error("Failed to load introspection");
  }

  m_serialize = rclcpp::get_service_typesupport_handle(
    m_type, "rosidl_typesupport_cpp",
    *rclcpp::get_typesupport_library(m_type, "rosidl_typesupport_cpp"));

  if (!m_serialize) {
    throw std::runtime_error("Failed to load serialization");
  }

  m_serviceMembers = static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(m_introspect->data);
  if (!m_serviceMembers) {
    throw std::runtime_error("Failed to load service members");
  }

  m_requestMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(m_serviceMembers->request_members_);
  if (!m_serviceMembers) {
    throw std::runtime_error("Failed to load request members");
  }

  m_responseMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(m_serviceMembers->response_members_);
  if (!m_serviceMembers) {
    throw std::runtime_error("Failed to load response members");
  }
}

json::value Client::Call(
  const json::value & data,
  std::chrono::seconds timeout)
{
  void * request = FillRequest(data);
  if (!request) {
    throw std::runtime_error("Failed to create request");
  }

  auto result = m_client->async_send_request(request).share();
  if (!(std::future_status::ready == result.wait_for(timeout))) {
    m_requestMembers->fini_function(request);
    free(request);
    throw std::runtime_error("Service call timeout");
  }

  if (!result.valid()) {
    throw std::runtime_error("Invalid response received");
  }

  try {
    void * response = result.get().get();
    if (!response) {
      throw std::runtime_error("Invalid response received");
    }

    return ExtractResponse(response);
  } catch (const std::exception & e) {
    throw std::runtime_error(std::string("Error while getting response: ") + e.what());
  }
}

void * Client::FillRequest(const json::value & data)
{
  void * request = malloc(m_requestMembers->size_of_);
  if (!request) {
    throw std::runtime_error("Memory allocation failed for request message");
  }

  m_requestMembers->init_function(request, rosidl_runtime_cpp::MessageInitialization::ALL);
  RecursiveFill(data.as_object(), request, m_requestMembers);
  return request;
}

void Client::RecursiveFill(
  const json::object & data,
  void * message,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    if (!data.contains(member.name_)) {
      throw std::runtime_error("Missing field: " + std::string(member.name_));
    }

    const json::value & value = data.at(member.name_);
    void * field = reinterpret_cast<char *>(message) + member.offset_;
    if (!field) {
      throw std::runtime_error("Failed to get field");
    }

    if (member.is_array_) {
      throw std::runtime_error("Array types not implemented");
    }

    if (member.members_ && member.members_->data) {
      const auto * subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
      RecursiveFill(value.as_object(), field, subMembers);
    } else {
      FillPrimitive(value, field, member);
    }
  }
}

void Client::FillPrimitive(
  const json::value & value,
  void * field,
  const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  using PrimitiveFunc = std::function<void()>;
  const std::unordered_map<int, PrimitiveFunc> primitives = {
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL, 
      [&value, &field]() {*reinterpret_cast<bool*>(field) = value.as_bool();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32, 
      [&value, &field]() {*reinterpret_cast<int32_t*>(field) = static_cast<int32_t>(value.as_int64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32, 
      [&value, &field]() {*reinterpret_cast<uint32_t*>(field) = static_cast<uint32_t>(value.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64, 
      [&value, &field]() {*reinterpret_cast<int64_t*>(field) = value.as_int64();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64, 
      [&value, &field]() {*reinterpret_cast<uint64_t*>(field) = value.as_uint64();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE, 
      [&value, &field]() {*reinterpret_cast<double*>(field) = value.as_double();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING, 
      [&value, &field]() {*reinterpret_cast<std::string*>(field) = std::string(value.as_string());}}
  };

  const auto it = primitives.find(member.type_id_);
  if (primitives.end() == it) {
    throw std::runtime_error("Unsupported primitive type: " + std::to_string(member.type_id_));
  }

  std::get<PrimitiveFunc>(*it)();
}

json::value Client::ExtractResponse(void * response)
{
  using HandlerFunc = std::function<json::value(void *)>;
  static const std::unordered_map<int, HandlerFunc> handlers = {{
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL, [](void * field) {
      return json::value(*reinterpret_cast<bool *>(field));}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32, [](void * field) {
      return json::value(*reinterpret_cast<int32_t *>(field));}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32, [](void * field) {
      return json::value(*reinterpret_cast<uint32_t *>(field));}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64, [](void * field) {
      return json::value(*reinterpret_cast<int64_t *>(field));}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64, [](void * field) {
      return json::value(*reinterpret_cast<uint64_t *>(field));}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE, [](void * field) {
      return json::value(*reinterpret_cast<double *>(field));}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING, [](void * field) {
      return json::value(*reinterpret_cast<std::string *>(field));}}
  }};

  json::object result;
  for (size_t i = 0; i < m_responseMembers->member_count_; ++i) {
    const auto & member = m_responseMembers->members_[i];
    const auto it = handlers.find(member.type_id_);
    if (handlers.end() == it){
      result[member.name_] = "<unsupported_type>";
      continue;
    }

    void * field = reinterpret_cast<char *>(response) + member.offset_;
    if (!field) {
      result[member.name_] = "<unsupported_type>";
      continue;
    }

    result[member.name_] = std::get<HandlerFunc>(*it)(field);
  }

  return result;
}

}  // namespace dynamic_interface
