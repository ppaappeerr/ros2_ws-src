// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from cartographer_ros_msgs:msg/StatusCode.idl
// generated code does not contain a copyright notice
#ifndef CARTOGRAPHER_ROS_MSGS__MSG__DETAIL__STATUS_CODE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CARTOGRAPHER_ROS_MSGS__MSG__DETAIL__STATUS_CODE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "cartographer_ros_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "cartographer_ros_msgs/msg/detail/status_code__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
bool cdr_serialize_cartographer_ros_msgs__msg__StatusCode(
  const cartographer_ros_msgs__msg__StatusCode * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
bool cdr_deserialize_cartographer_ros_msgs__msg__StatusCode(
  eprosima::fastcdr::Cdr &,
  cartographer_ros_msgs__msg__StatusCode * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t get_serialized_size_cartographer_ros_msgs__msg__StatusCode(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t max_serialized_size_cartographer_ros_msgs__msg__StatusCode(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
bool cdr_serialize_key_cartographer_ros_msgs__msg__StatusCode(
  const cartographer_ros_msgs__msg__StatusCode * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t get_serialized_size_key_cartographer_ros_msgs__msg__StatusCode(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t max_serialized_size_key_cartographer_ros_msgs__msg__StatusCode(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cartographer_ros_msgs, msg, StatusCode)();

#ifdef __cplusplus
}
#endif

#endif  // CARTOGRAPHER_ROS_MSGS__MSG__DETAIL__STATUS_CODE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
