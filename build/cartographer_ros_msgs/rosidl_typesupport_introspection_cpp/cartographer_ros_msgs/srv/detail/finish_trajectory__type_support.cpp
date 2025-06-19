// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cartographer_ros_msgs:srv/FinishTrajectory.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cartographer_ros_msgs/srv/detail/finish_trajectory__functions.h"
#include "cartographer_ros_msgs/srv/detail/finish_trajectory__struct.hpp"
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

void FinishTrajectory_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cartographer_ros_msgs::srv::FinishTrajectory_Request(_init);
}

void FinishTrajectory_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cartographer_ros_msgs::srv::FinishTrajectory_Request *>(message_memory);
  typed_message->~FinishTrajectory_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FinishTrajectory_Request_message_member_array[1] = {
  {
    "trajectory_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::FinishTrajectory_Request, trajectory_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FinishTrajectory_Request_message_members = {
  "cartographer_ros_msgs::srv",  // message namespace
  "FinishTrajectory_Request",  // message name
  1,  // number of fields
  sizeof(cartographer_ros_msgs::srv::FinishTrajectory_Request),
  false,  // has_any_key_member_
  FinishTrajectory_Request_message_member_array,  // message members
  FinishTrajectory_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  FinishTrajectory_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FinishTrajectory_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FinishTrajectory_Request_message_members,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_hash,
  &cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_description,
  &cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Request>()
{
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, FinishTrajectory_Request)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_Request_message_type_support_handle;
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
// #include "cartographer_ros_msgs/srv/detail/finish_trajectory__functions.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/finish_trajectory__struct.hpp"
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

void FinishTrajectory_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cartographer_ros_msgs::srv::FinishTrajectory_Response(_init);
}

void FinishTrajectory_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cartographer_ros_msgs::srv::FinishTrajectory_Response *>(message_memory);
  typed_message->~FinishTrajectory_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FinishTrajectory_Response_message_member_array[1] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::msg::StatusResponse>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::FinishTrajectory_Response, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FinishTrajectory_Response_message_members = {
  "cartographer_ros_msgs::srv",  // message namespace
  "FinishTrajectory_Response",  // message name
  1,  // number of fields
  sizeof(cartographer_ros_msgs::srv::FinishTrajectory_Response),
  false,  // has_any_key_member_
  FinishTrajectory_Response_message_member_array,  // message members
  FinishTrajectory_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  FinishTrajectory_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FinishTrajectory_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FinishTrajectory_Response_message_members,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_hash,
  &cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_description,
  &cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Response>()
{
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, FinishTrajectory_Response)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_Response_message_type_support_handle;
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
// #include "cartographer_ros_msgs/srv/detail/finish_trajectory__functions.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/finish_trajectory__struct.hpp"
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

void FinishTrajectory_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cartographer_ros_msgs::srv::FinishTrajectory_Event(_init);
}

void FinishTrajectory_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cartographer_ros_msgs::srv::FinishTrajectory_Event *>(message_memory);
  typed_message->~FinishTrajectory_Event();
}

size_t size_function__FinishTrajectory_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FinishTrajectory_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__FinishTrajectory_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__FinishTrajectory_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cartographer_ros_msgs::srv::FinishTrajectory_Request *>(
    get_const_function__FinishTrajectory_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<cartographer_ros_msgs::srv::FinishTrajectory_Request *>(untyped_value);
  value = item;
}

void assign_function__FinishTrajectory_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cartographer_ros_msgs::srv::FinishTrajectory_Request *>(
    get_function__FinishTrajectory_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const cartographer_ros_msgs::srv::FinishTrajectory_Request *>(untyped_value);
  item = value;
}

void resize_function__FinishTrajectory_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__FinishTrajectory_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__FinishTrajectory_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__FinishTrajectory_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__FinishTrajectory_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const cartographer_ros_msgs::srv::FinishTrajectory_Response *>(
    get_const_function__FinishTrajectory_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<cartographer_ros_msgs::srv::FinishTrajectory_Response *>(untyped_value);
  value = item;
}

void assign_function__FinishTrajectory_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<cartographer_ros_msgs::srv::FinishTrajectory_Response *>(
    get_function__FinishTrajectory_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const cartographer_ros_msgs::srv::FinishTrajectory_Response *>(untyped_value);
  item = value;
}

void resize_function__FinishTrajectory_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<cartographer_ros_msgs::srv::FinishTrajectory_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember FinishTrajectory_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::FinishTrajectory_Event, info),  // bytes offset in struct
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
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::FinishTrajectory_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__FinishTrajectory_Event__request,  // size() function pointer
    get_const_function__FinishTrajectory_Event__request,  // get_const(index) function pointer
    get_function__FinishTrajectory_Event__request,  // get(index) function pointer
    fetch_function__FinishTrajectory_Event__request,  // fetch(index, &value) function pointer
    assign_function__FinishTrajectory_Event__request,  // assign(index, value) function pointer
    resize_function__FinishTrajectory_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(cartographer_ros_msgs::srv::FinishTrajectory_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__FinishTrajectory_Event__response,  // size() function pointer
    get_const_function__FinishTrajectory_Event__response,  // get_const(index) function pointer
    get_function__FinishTrajectory_Event__response,  // get(index) function pointer
    fetch_function__FinishTrajectory_Event__response,  // fetch(index, &value) function pointer
    assign_function__FinishTrajectory_Event__response,  // assign(index, value) function pointer
    resize_function__FinishTrajectory_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers FinishTrajectory_Event_message_members = {
  "cartographer_ros_msgs::srv",  // message namespace
  "FinishTrajectory_Event",  // message name
  3,  // number of fields
  sizeof(cartographer_ros_msgs::srv::FinishTrajectory_Event),
  false,  // has_any_key_member_
  FinishTrajectory_Event_message_member_array,  // message members
  FinishTrajectory_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  FinishTrajectory_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t FinishTrajectory_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FinishTrajectory_Event_message_members,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_hash,
  &cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_description,
  &cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Event>()
{
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, FinishTrajectory_Event)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_Event_message_type_support_handle;
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
// #include "cartographer_ros_msgs/srv/detail/finish_trajectory__functions.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/finish_trajectory__struct.hpp"
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
static ::rosidl_typesupport_introspection_cpp::ServiceMembers FinishTrajectory_service_members = {
  "cartographer_ros_msgs::srv",  // service namespace
  "FinishTrajectory",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t FinishTrajectory_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &FinishTrajectory_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<cartographer_ros_msgs::srv::FinishTrajectory>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<cartographer_ros_msgs::srv::FinishTrajectory>,
  &cartographer_ros_msgs__srv__FinishTrajectory__get_type_hash,
  &cartographer_ros_msgs__srv__FinishTrajectory__get_type_description,
  &cartographer_ros_msgs__srv__FinishTrajectory__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace cartographer_ros_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::cartographer_ros_msgs::srv::rosidl_typesupport_introspection_cpp::FinishTrajectory_service_type_support_handle;
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
        ::cartographer_ros_msgs::srv::FinishTrajectory_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cartographer_ros_msgs::srv::FinishTrajectory_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::cartographer_ros_msgs::srv::FinishTrajectory_Event
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
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cartographer_ros_msgs, srv, FinishTrajectory)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<cartographer_ros_msgs::srv::FinishTrajectory>();
}

#ifdef __cplusplus
}
#endif
