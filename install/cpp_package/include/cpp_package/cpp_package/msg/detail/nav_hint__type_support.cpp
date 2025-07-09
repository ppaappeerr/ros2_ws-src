// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "cpp_package/msg/detail/nav_hint__functions.h"
#include "cpp_package/msg/detail/nav_hint__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace cpp_package
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void NavHint_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) cpp_package::msg::NavHint(_init);
}

void NavHint_fini_function(void * message_memory)
{
  auto typed_message = static_cast<cpp_package::msg::NavHint *>(message_memory);
  typed_message->~NavHint();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NavHint_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp_package::msg::NavHint, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "action",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(cpp_package::msg::NavHint, action),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NavHint_message_members = {
  "cpp_package::msg",  // message namespace
  "NavHint",  // message name
  2,  // number of fields
  sizeof(cpp_package::msg::NavHint),
  false,  // has_any_key_member_
  NavHint_message_member_array,  // message members
  NavHint_init_function,  // function to initialize message memory (memory has to be allocated)
  NavHint_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NavHint_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NavHint_message_members,
  get_message_typesupport_handle_function,
  &cpp_package__msg__NavHint__get_type_hash,
  &cpp_package__msg__NavHint__get_type_description,
  &cpp_package__msg__NavHint__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace cpp_package


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cpp_package::msg::NavHint>()
{
  return &::cpp_package::msg::rosidl_typesupport_introspection_cpp::NavHint_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cpp_package, msg, NavHint)() {
  return &::cpp_package::msg::rosidl_typesupport_introspection_cpp::NavHint_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
