// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice
#ifndef CPP_PACKAGE__MSG__DETAIL__NAV_HINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CPP_PACKAGE__MSG__DETAIL__NAV_HINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "cpp_package/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "cpp_package/msg/detail/nav_hint__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
bool cdr_serialize_cpp_package__msg__NavHint(
  const cpp_package__msg__NavHint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
bool cdr_deserialize_cpp_package__msg__NavHint(
  eprosima::fastcdr::Cdr &,
  cpp_package__msg__NavHint * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
size_t get_serialized_size_cpp_package__msg__NavHint(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
size_t max_serialized_size_cpp_package__msg__NavHint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
bool cdr_serialize_key_cpp_package__msg__NavHint(
  const cpp_package__msg__NavHint * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
size_t get_serialized_size_key_cpp_package__msg__NavHint(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
size_t max_serialized_size_key_cpp_package__msg__NavHint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cpp_package
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cpp_package, msg, NavHint)();

#ifdef __cplusplus
}
#endif

#endif  // CPP_PACKAGE__MSG__DETAIL__NAV_HINT__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
