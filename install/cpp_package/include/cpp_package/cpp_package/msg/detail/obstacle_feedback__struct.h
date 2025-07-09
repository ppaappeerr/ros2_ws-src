// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/obstacle_feedback.h"


#ifndef CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__STRUCT_H_
#define CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__STRUCT_H_

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

/// Struct defined in msg/ObstacleFeedback in the package cpp_package.
typedef struct cpp_package__msg__ObstacleFeedback
{
  std_msgs__msg__Header header;
  float min_left;
  float min_center;
  float min_right;
  /// 0=Safe,1=Warn,2=Danger
  uint8_t level_left;
  uint8_t level_center;
  uint8_t level_right;
} cpp_package__msg__ObstacleFeedback;

// Struct for a sequence of cpp_package__msg__ObstacleFeedback.
typedef struct cpp_package__msg__ObstacleFeedback__Sequence
{
  cpp_package__msg__ObstacleFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cpp_package__msg__ObstacleFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__STRUCT_H_
