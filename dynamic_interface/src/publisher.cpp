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

void Publisher::Publish(const boost::json::value & data)
{
  if (!m_members || !m_publisher) {
    RCLCPP_ERROR(m_node->get_logger(), "Publisher not initialized properly");
    return;
  }

  struct MessageDeleter
  {
    const rosidl_typesupport_introspection_cpp::MessageMembers * Members;
    void operator()(void * ptr) const noexcept
    {
      if (ptr && Members) {
        Members->fini_function(ptr);
        free(ptr);
      }
    }
  };

  void * message = malloc(m_members->size_of_);
  if (!message) {
    RCLCPP_ERROR(m_node->get_logger(), "Memory allocation failed for message");
    return;
  }

  m_members->init_function(message, rosidl_runtime_cpp::MessageInitialization::ALL);
  std::unique_ptr<void, MessageDeleter> message_guard(message, MessageDeleter{m_members});

  try {
    RecursiveData(data.as_object(), message, m_members);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(m_node->get_logger(), "JSON parse error: %s", e.what());
    return;
  }

  rclcpp::SerializedMessage serializedMessage;
  const auto ret = rmw_serialize(message, m_serialize, &serializedMessage.get_rcl_serialized_message());
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
  const boost::json::object & data,
  void * message,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members)
{
  if (!members || !message) {
    throw std::runtime_error("Invalid members or message pointer");
  }

  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const std::string & name = member.name_;

    const auto it = data.find(name);
    if (data.end() == it) {
      throw std::runtime_error("JSON missing field: " + name);
    }

    const boost::json::value & fieldJson = it->value();
    void * field = reinterpret_cast<char *>(message) + member.offset_;
    if (!field) {
      throw std::runtime_error("Invalid field offset for: " + name);
    }

    if (member.is_array_) {
      if (!fieldJson.is_array()) {
        throw std::runtime_error("Field " + name + " should be an array");
      }

      const auto & arr = fieldJson.as_array();
      if (member.members_ && member.members_->data) {
        const auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
        for (size_t idx = 0; idx < arr.size(); ++idx) {
          RecursiveData(arr[idx].as_object(), reinterpret_cast<char *>(field) + idx * subMembers->size_of_, subMembers);
        }

        continue;
      }

      size_t primitiveSize = GetPrimitiveSize(member);
      for (size_t idx = 0; idx < arr.size(); ++idx) {
        SetPrimitiveField(arr[idx], reinterpret_cast<char *>(field) + idx * primitiveSize, member);
      }
      continue;
    }

    if (member.members_ && member.members_->data) {
      const auto subMembers = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
      RecursiveData(fieldJson.as_object(), field, subMembers);
      continue;
    }

    SetPrimitiveField(fieldJson, field, member);
  }
}

void Publisher::SetPrimitiveField(
  const boost::json::value & value,
  void * field,
  const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  using SetFieldFunc = std::function<void(void *, const boost::json::value &)>;
  static const std::unordered_map<uint8_t, SetFieldFunc> setters = {
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<bool *>(f) = v.as_bool();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<uint8_t *>(f) = static_cast<uint8_t>(v.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<uint8_t *>(f) = static_cast<uint8_t>(v.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<int8_t *>(f) = static_cast<uint8_t>(v.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<int8_t *>(f) = static_cast<uint8_t>(v.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<uint16_t *>(f) = static_cast<uint16_t>(v.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<int16_t *>(f) = static_cast<int16_t>(v.as_int64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<uint32_t *>(f) = static_cast<uint32_t>(v.as_uint64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<int32_t *>(f) = static_cast<int32_t>(v.as_int64());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<uint64_t *>(f) = v.as_uint64();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<int64_t *>(f) = v.as_int64();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<float *>(f) = static_cast<float>(v.as_double());}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<double *>(f) = v.as_double();}},

    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,
      [](void * f, const boost::json::value & v) {
        *reinterpret_cast<std::string *>(f) = v.as_string().c_str();}}
  };

  const auto it = setters.find(member.type_id_);
  if (setters.end() == it) {
    throw std::runtime_error("Unsupported type_id: " + std::to_string(member.type_id_));
  }

  std::get<SetFieldFunc>(*it)(field, value);
}

size_t Publisher::GetPrimitiveSize(const rosidl_typesupport_introspection_cpp::MessageMember & member) const
{
  static const std::unordered_map<uint8_t, size_t> typeSizes = {
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

  const auto it = typeSizes.find(member.type_id_);
  if (typeSizes.end() == it) {
    throw std::runtime_error(
      "Unknown primitive size for type_id: " + std::to_string(member.type_id_));
  }

  return std::get<size_t>(*it);
}

}  // namespace dynamic_interface
