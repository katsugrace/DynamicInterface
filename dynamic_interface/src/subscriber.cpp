// Copyright (c) 2025 Victor Drobizov <drobizov.victor@gmail.com>

#include <rosidl_typesupport_introspection_cpp/field_types.hpp>

#include <dynamic_interface/subscriber.hpp>

namespace dynamic_interface
{

namespace json = boost::json;

Subscriber::Subscriber(
  std::shared_ptr<rclcpp::Node> node,
  std::string name,
  std::string type,
  CallbackFunc callback,
  rclcpp::QoS qos)
: m_node(std::move(node))
, m_name(std::move(name))
, m_type(std::move(type))
, m_callback(std::move(callback))
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

  if (!m_callback) {
    throw std::runtime_error("Callback is null");
  }

  m_subscription = m_node->create_generic_subscription(
    m_name,
    m_type,
    qos,
    [this](std::shared_ptr<rclcpp::SerializedMessage> message) {
      this->OnMessage(*message);});

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

void Subscriber::OnMessage(const rclcpp::SerializedMessage & data)
{
  void * message = malloc(m_members->size_of_);
  if (!message) {
    RCLCPP_ERROR(m_node->get_logger(), "Memory allocation failed");
    return;
  }

  m_members->init_function(message, rosidl_runtime_cpp::MessageInitialization::ALL);
  struct MessageDeleter
  {
    const rosidl_typesupport_introspection_cpp::MessageMembers * members;
    void operator()(void * ptr) const noexcept
    {
      if (ptr && members) {
        members->fini_function(ptr);
        free(ptr);
      }
    }
  };

  std::unique_ptr<void, MessageDeleter> guard(message, MessageDeleter{m_members});
  if (!(rmw_deserialize(&data.get_rcl_serialized_message(), m_serialize, message) == RMW_RET_OK)) {
    RCLCPP_ERROR(m_node->get_logger(), "Deserialization failed");
    return;
  }

  try {
    json::object data;
    ToJson(message, m_members, data);
    m_callback(data);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(m_node->get_logger(), "ToJson error: %s", e.what());
  }
}

void Subscriber::ToJson(
  void * message,
  const rosidl_typesupport_introspection_cpp::MessageMembers * members,
  json::object & outData)
{
  for (size_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const std::string & name = member.name_;
    void * field = reinterpret_cast<char *>(message) + member.offset_;
    if (member.is_array_) {
      json::array arr;
      size_t size = member.array_size_;
      if (member.members_ && member.members_->data) {
        const auto * sub = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
        for (size_t j = 0; j < size; ++j) {
          json::object subJson;
          ToJson(reinterpret_cast<char *>(field) + j * sub->size_of_, sub, subJson);
          arr.push_back(subJson);
        }
      } else {
        for (size_t j = 0; j < size; ++j) {
          arr.push_back(ExtractPrimitive(reinterpret_cast<char *>(field) + j * GetPrimitiveSize(member), member));
        }
      }

      outData[name] = arr;
    } else {
      if (member.members_ && member.members_->data) {
        json::object subJson;
        const auto * sub = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member.members_->data);
        ToJson(field, sub, subJson);
        outData[name] = subJson;
      } else {
        outData[name] = ExtractPrimitive(field, member);
      }
    }
  }
}

json::value Subscriber::ExtractPrimitive(void * field, const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  using Extractor = std::function<json::value(void*)>;
  static const std::unordered_map<uint8_t, Extractor> extractors = {
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL, [](void * f) {return json::value(*reinterpret_cast<bool *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE, [](void * f) {return json::value(*reinterpret_cast<uint8_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8, [](void * f) {return json::value(*reinterpret_cast<uint8_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR, [](void * f) {return json::value(*reinterpret_cast<int8_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8, [](void * f) {return json::value(*reinterpret_cast<int8_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16, [](void * f) {return json::value(*reinterpret_cast<uint16_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16, [](void * f) {return json::value(*reinterpret_cast<int16_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32, [](void * f) {return json::value(*reinterpret_cast<uint32_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32, [](void * f) {return json::value(*reinterpret_cast<int32_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64, [](void * f) {return json::value(*reinterpret_cast<uint64_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64, [](void * f) {return json::value(*reinterpret_cast<int64_t *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT, [](void * f) {return json::value(*reinterpret_cast<float *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE, [](void * f) {return json::value(*reinterpret_cast<double *>(f));}},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING, [](void * f) {return json::value(*reinterpret_cast<std::string *>(f));}},
  };

  const auto it = extractors.find(member.type_id_);
  if (extractors.end() == it) {
    throw std::runtime_error("Unsupported type in ExtractPrimitive");
  }

  return std::get<Extractor>(*it)(field);
}

size_t Subscriber::GetPrimitiveSize(const rosidl_typesupport_introspection_cpp::MessageMember & member)
{
  static const std::unordered_map<uint8_t, size_t> typeSizes = {
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL, sizeof(bool)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE, sizeof(uint8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8, sizeof(uint8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR, sizeof(int8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8, sizeof(int8_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16, sizeof(uint16_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16, sizeof(int16_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32, sizeof(uint32_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32, sizeof(int32_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64, sizeof(uint64_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64, sizeof(int64_t)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT, sizeof(float)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE, sizeof(double)},
    {rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING, sizeof(std::string)}
  };

  const auto it = typeSizes.find(member.type_id_);
  if (typeSizes.end() == it) {
    throw std::runtime_error("Unknown type size for type_id: " + std::to_string(member.type_id_));
  }

  return std::get<size_t>(*it);
}

}  // namespace dynamic_interface
