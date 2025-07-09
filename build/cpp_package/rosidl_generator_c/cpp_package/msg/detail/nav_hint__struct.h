// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/nav_hint.h"


#ifndef CPP_PACKAGE__MSG__DETAIL__NAV_HINT__STRUCT_H_
#define CPP_PACKAGE__MSG__DETAIL__NAV_HINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'action'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/NavHint in the package cpp_package.
typedef struct cpp_package__msg__NavHint
{
  std_msgs__msg__Header header;
  /// GO | TURN_LEFT | TURN_RIGHT | SLOW | STOP
  rosidl_runtime_c__String action;
} cpp_package__msg__NavHint;

// Struct for a sequence of cpp_package__msg__NavHint.
typedef struct cpp_package__msg__NavHint__Sequence
{
  cpp_package__msg__NavHint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_package__msg__NavHint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CPP_PACKAGE__MSG__DETAIL__NAV_HINT__STRUCT_H_
