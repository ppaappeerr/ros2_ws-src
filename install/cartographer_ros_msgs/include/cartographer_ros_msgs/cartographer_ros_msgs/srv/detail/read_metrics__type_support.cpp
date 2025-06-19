// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cartographer_ros_msgs:srv/ReadMetrics.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cartographer_ros_msgs/srv/detail/read_metrics__functions.h"
#include "cartographer_ros_msgs/srv/detail/read_metrics__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ReadMetrics_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cartographer_ros_msgs::srv::ReadMetrics_Request(_init);
}

void ReadMetrics_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cartographer_ros_msgs::srv::ReadMetrics_Request *>(message_memory);
  typed_message->~ReadMetrics_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReadMetrics_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReadMetrics_Request_message_members = {
  "cartographer_ros_msgs::srv",  // message namespace
  "ReadMetrics_Request",  // message name
  1,  // number of fields
  sizeof(cartographer_ros_msgs::srv::ReadMetrics_Request),
  false,  // has_any_key_member_
  ReadMetrics_Request_message_member_array,  // message members
  ReadMetrics_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ReadMetrics_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReadMetrics_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReadMetrics_Request_message_members,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_hash,
  &cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_description,
  &cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Request>()
{
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, ReadMetrics_Request)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/read_metrics__functions.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/read_metrics__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ReadMetrics_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cartographer_ros_msgs::srv::ReadMetrics_Response(_init);
}

void ReadMetrics_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cartographer_ros_msgs::srv::ReadMetrics_Response *>(message_memory);
  typed_message->~ReadMetrics_Response();
}

size_t size_function__ReadMetrics_Response__metric_families(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cartographer_ros_msgs::msg::MetricFamily> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReadMetrics_Response__metric_families(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cartographer_ros_msgs::msg::MetricFamily> *>(untyped_member);
  return &member[index];
}

void * get_function__ReadMetrics_Response__metric_families(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cartographer_ros_msgs::msg::MetricFamily> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReadMetrics_Response__metric_families(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cartographer_ros_msgs::msg::MetricFamily *>(
    get_const_function__ReadMetrics_Response__metric_families(untyped_member, index));
  auto & value = *reinterpret_cast<cartographer_ros_msgs::msg::MetricFamily *>(untyped_value);
  value = item;
}

void assign_function__ReadMetrics_Response__metric_families(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cartographer_ros_msgs::msg::MetricFamily *>(
    get_function__ReadMetrics_Response__metric_families(untyped_member, index));
  const auto & value = *reinterpret_cast<const cartographer_ros_msgs::msg::MetricFamily *>(untyped_value);
  item = value;
}

void resize_function__ReadMetrics_Response__metric_families(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cartographer_ros_msgs::msg::MetricFamily> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReadMetrics_Response_message_member_array[3] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::msg::StatusResponse>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Response, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "metric_families",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::msg::MetricFamily>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Response, metric_families),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReadMetrics_Response__metric_families,  // size() function pointer
    get_const_function__ReadMetrics_Response__metric_families,  // get_const(index) function pointer
    get_function__ReadMetrics_Response__metric_families,  // get(index) function pointer
    fetch_function__ReadMetrics_Response__metric_families,  // fetch(index, &value) function pointer
    assign_function__ReadMetrics_Response__metric_families,  // assign(index, value) function pointer
    resize_function__ReadMetrics_Response__metric_families  // resize(index) function pointer
  },
  {
    "timestamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Response, timestamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReadMetrics_Response_message_members = {
  "cartographer_ros_msgs::srv",  // message namespace
  "ReadMetrics_Response",  // message name
  3,  // number of fields
  sizeof(cartographer_ros_msgs::srv::ReadMetrics_Response),
  false,  // has_any_key_member_
  ReadMetrics_Response_message_member_array,  // message members
  ReadMetrics_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ReadMetrics_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReadMetrics_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReadMetrics_Response_message_members,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_hash,
  &cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_description,
  &cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Response>()
{
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, ReadMetrics_Response)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/read_metrics__functions.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/read_metrics__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void ReadMetrics_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cartographer_ros_msgs::srv::ReadMetrics_Event(_init);
}

void ReadMetrics_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cartographer_ros_msgs::srv::ReadMetrics_Event *>(message_memory);
  typed_message->~ReadMetrics_Event();
}

size_t size_function__ReadMetrics_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::ReadMetrics_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReadMetrics_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::ReadMetrics_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__ReadMetrics_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cartographer_ros_msgs::srv::ReadMetrics_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReadMetrics_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cartographer_ros_msgs::srv::ReadMetrics_Request *>(
    get_const_function__ReadMetrics_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<cartographer_ros_msgs::srv::ReadMetrics_Request *>(untyped_value);
  value = item;
}

void assign_function__ReadMetrics_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cartographer_ros_msgs::srv::ReadMetrics_Request *>(
    get_function__ReadMetrics_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const cartographer_ros_msgs::srv::ReadMetrics_Request *>(untyped_value);
  item = value;
}

void resize_function__ReadMetrics_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cartographer_ros_msgs::srv::ReadMetrics_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ReadMetrics_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::ReadMetrics_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ReadMetrics_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::ReadMetrics_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__ReadMetrics_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cartographer_ros_msgs::srv::ReadMetrics_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__ReadMetrics_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cartographer_ros_msgs::srv::ReadMetrics_Response *>(
    get_const_function__ReadMetrics_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<cartographer_ros_msgs::srv::ReadMetrics_Response *>(untyped_value);
  value = item;
}

void assign_function__ReadMetrics_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cartographer_ros_msgs::srv::ReadMetrics_Response *>(
    get_function__ReadMetrics_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const cartographer_ros_msgs::srv::ReadMetrics_Response *>(untyped_value);
  item = value;
}

void resize_function__ReadMetrics_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cartographer_ros_msgs::srv::ReadMetrics_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ReadMetrics_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReadMetrics_Event__request,  // size() function pointer
    get_const_function__ReadMetrics_Event__request,  // get_const(index) function pointer
    get_function__ReadMetrics_Event__request,  // get(index) function pointer
    fetch_function__ReadMetrics_Event__request,  // fetch(index, &value) function pointer
    assign_function__ReadMetrics_Event__request,  // assign(index, value) function pointer
    resize_function__ReadMetrics_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::ReadMetrics_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__ReadMetrics_Event__response,  // size() function pointer
    get_const_function__ReadMetrics_Event__response,  // get_const(index) function pointer
    get_function__ReadMetrics_Event__response,  // get(index) function pointer
    fetch_function__ReadMetrics_Event__response,  // fetch(index, &value) function pointer
    assign_function__ReadMetrics_Event__response,  // assign(index, value) function pointer
    resize_function__ReadMetrics_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ReadMetrics_Event_message_members = {
  "cartographer_ros_msgs::srv",  // message namespace
  "ReadMetrics_Event",  // message name
  3,  // number of fields
  sizeof(cartographer_ros_msgs::srv::ReadMetrics_Event),
  false,  // has_any_key_member_
  ReadMetrics_Event_message_member_array,  // message members
  ReadMetrics_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  ReadMetrics_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ReadMetrics_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReadMetrics_Event_message_members,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_hash,
  &cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_description,
  &cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Event>()
{
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, ReadMetrics_Event)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/read_metrics__functions.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/read_metrics__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers ReadMetrics_service_members = {
  "cartographer_ros_msgs::srv",  // service namespace
  "ReadMetrics",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t ReadMetrics_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ReadMetrics_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<cartographer_ros_msgs::srv::ReadMetrics>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<cartographer_ros_msgs::srv::ReadMetrics>,
  &cartographer_ros_msgs__srv__ReadMetrics__get_type_hash,
  &cartographer_ros_msgs__srv__ReadMetrics__get_type_description,
  &cartographer_ros_msgs__srv__ReadMetrics__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::ReadMetrics_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cartographer_ros_msgs::srv::ReadMetrics_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cartographer_ros_msgs::srv::ReadMetrics_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cartographer_ros_msgs::srv::ReadMetrics_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, ReadMetrics)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<cartographer_ros_msgs::srv::ReadMetrics>();
}

#ifdef __cplusplus
}
#endif
