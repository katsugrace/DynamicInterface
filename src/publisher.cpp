// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <dynamic_interface/publisher.hpp>

namespace dynamic_interface
{

Publisher::Publisher(
  std::shared_ptr<rclcpp::Node> node,
  std::string name,
  std::string type,
  rclcpp::QoS qos)
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

  m_publisher = m_node->create_generic_publisher(m_name, m_type, qos);

  m_introspect = rclcpp::get_message_typesupport_handle(
    m_type,
    "rosidl_typesupport_introspection_cpp",
    *rclcpp::get_typesupport_library(m_type, "rosidl_typesupport_introspection_cpp"));

  if (!m_introspect) {
    throw std::runtime_error("Failed to load introspection");
  }

  m_serialize = rclcpp::get_message_typesupport_handle(
    m_type,
    "rosidl_typesupport_cpp",
    *rclcpp::get_typesupport_library(m_type, "rosidl_typesupport_cpp"));

  if (!m_serialize) {
    throw std::runtime_error("Failed to load serialization");
  }

  m_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(m_introspect->data);
  if (!m_members) {
    throw std::runtime_error("Introspection has no members data");
  }
}

void Publisher::Publish(const nlohmann::json &data)
{
  if (!m_members || !m_publisher) {
    RCLCPP_ERROR(m_node->get_logger(), "Publisher not initialized properly");
    return;
  }

  struct MessageDeleter
  {
    const rosidl_typesupport_introspection_cpp::MessageMembers * Members;
    void operator()(void *ptr) const noexcept
    {
      if (ptr && Members) {
        Members->fini_function(ptr);
        free(ptr);
      }
    }
  };

  void * raw_message = malloc(m_members->size_of_);
  if (!raw_message) {
    RCLCPP_ERROR(m_node->get_logger(), "Memory allocation failed for message");
    return;
  }

  m_members->init_function(raw_message, rosidl_runtime_cpp::MessageInitialization::ALL);
  std::unique_ptr<void, MessageDeleter> message_guard(raw_message, MessageDeleter{m_members});

  try {
    RecursiveData(data, raw_message, m_members);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(m_node->get_logger(), "JSON parse error: %s", e.what());
    return;
  }

  rclcpp::SerializedMessage serializedMessage;
  rmw_ret_t ret = rmw_serialize(raw_message, m_serialize, &serializedMessage.get_rcl_serialized_message());
  if (!(RMW_RET_OK == ret)) {
    RCLCPP_ERROR(m_node->get_logger(), "rmw_serialize failed with code %d", static_cast<int>(ret));
    return;
  }

  try {
    m_publisher->publish(serializedMessage);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(m_node->get_logger(), "Failed to publish message: %s", e.what());
  }
}

void Publisher::RecursiveData(
  const nlohmann::json & data,
  void * message,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  if (!members || !message) {
    throw std::runtime_error("Invalid members or message pointer");
  }

  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const std::string & name = member.name_;

    if (!data.contains(name)) {
      throw std::runtime_error("JSON missing field: " + name);
    }

    const nlohmann::json & fieldJson = data.at(name);

    void * field = reinterpret_cast<char *>(message) + member.offset_;
    if (!field) {
      throw std::runtime_error("Invalid field offset for: " + name);
    }

    if (member.is_array_) {
      if (!fieldJson.is_array()) {
        throw std::runtime_error("Field " + name + " should be an array");
      }

      if (member.members_ && member.members_->data) {
        auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
        for (size_t idx = 0; idx < fieldJson.size(); ++idx) {
          RecursiveData(
            fieldJson.at(idx),
            reinterpret_cast<char *>(field) + idx * subMembers->size_of_,
            subMembers);
        }
        continue;
      }

      size_t primitiveSize = GetPrimitiveSize(member);
      for (size_t idx = 0; idx < fieldJson.size(); ++idx) {
        SetPrimitiveField(
          fieldJson.at(idx),
          reinterpret_cast<char *>(field) + idx * primitiveSize,
          member);
      }

      continue;
    }

    if (member.members_ && member.members_->data) {
      auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
      RecursiveData(fieldJson, field, subMembers);
      continue;
    }

    SetPrimitiveField(fieldJson, field, member);
  }
}

void Publisher::SetPrimitiveField(
  const nlohmann::json & value,
  void * field,
  const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  using SetFieldFunc = std::function<void(void *, const nlohmann::json &)>;
  static const std::unordered_map<uint8_t, SetFieldFunc> setters = {
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL,    [](void * f, const nlohmann::json & v) { *reinterpret_cast<bool *>(f) = v.get<bool>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE,    [](void * f, const nlohmann::json & v) { *reinterpret_cast<uint8_t *>(f) = v.get<uint8_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,   [](void * f, const nlohmann::json & v) { *reinterpret_cast<uint8_t *>(f) = v.get<uint8_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR,    [](void * f, const nlohmann::json & v) { *reinterpret_cast<int8_t *>(f) = v.get<int8_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,    [](void * f, const nlohmann::json & v) { *reinterpret_cast<int8_t *>(f) = v.get<int8_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  [](void * f, const nlohmann::json & v) { *reinterpret_cast<uint16_t *>(f) = v.get<uint16_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,   [](void * f, const nlohmann::json & v) { *reinterpret_cast<int16_t *>(f) = v.get<int16_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  [](void * f, const nlohmann::json & v) { *reinterpret_cast<uint32_t *>(f) = v.get<uint32_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,   [](void * f, const nlohmann::json & v) { *reinterpret_cast<int32_t *>(f) = v.get<int32_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  [](void * f, const nlohmann::json & v) { *reinterpret_cast<uint64_t *>(f) = v.get<uint64_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,   [](void * f, const nlohmann::json & v) { *reinterpret_cast<int64_t *>(f) = v.get<int64_t>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,   [](void * f, const nlohmann::json & v) { *reinterpret_cast<float *>(f) = v.get<float>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  [](void * f, const nlohmann::json & v) { *reinterpret_cast<double *>(f) = v.get<double>(); }},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  [](void * f, const nlohmann::json & v) { *reinterpret_cast<std::string *>(f) = v.get<std::string>(); }},
  };

  const auto it = setters.find(member.type_id_);
  if (it == setters.end()) {
    throw std::runtime_error("Unsupported type_id: " + std::to_string(member.type_id_));
  }

  std::get<SetFieldFunc>(*it)(field, value);
}

size_t Publisher::GetPrimitiveSize(const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  static const std::unordered_map<uint8_t, size_t> type_size_map = {
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL,    sizeof(bool)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE,    sizeof(uint8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,   sizeof(uint8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR,    sizeof(int8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,    sizeof(int8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,  sizeof(uint16_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,   sizeof(int16_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  sizeof(uint32_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,   sizeof(int32_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  sizeof(uint64_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,   sizeof(int64_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,   sizeof(float)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  sizeof(double)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  sizeof(std::string)}
  };

  const auto it = type_size_map.find(member.type_id_);
  if (type_size_map.end() == it) {
    throw std::runtime_error(
      "Unknown primitive size for type_id: " + std::to_string(member.type_id_));
  }

  return std::get<size_t>(*it);
}

}  // namespace dynamic_interface
