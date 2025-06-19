// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice
#include "lio_sam/msg/detail/cloud_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "lio_sam/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "lio_sam/msg/detail/cloud_info__struct.h"
#include "lio_sam/msg/detail/cloud_info__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // end_ring_index, point_col_ind, point_range, start_ring_index
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // end_ring_index, point_col_ind, point_range, start_ring_index
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"  // cloud_corner, cloud_deskewed, cloud_surface
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
bool cdr_serialize_sensor_msgs__msg__PointCloud2(
  const sensor_msgs__msg__PointCloud2 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
bool cdr_deserialize_sensor_msgs__msg__PointCloud2(
  eprosima::fastcdr::Cdr & cdr,
  sensor_msgs__msg__PointCloud2 * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t get_serialized_size_sensor_msgs__msg__PointCloud2(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t max_serialized_size_sensor_msgs__msg__PointCloud2(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
bool cdr_serialize_key_sensor_msgs__msg__PointCloud2(
  const sensor_msgs__msg__PointCloud2 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t get_serialized_size_key_sensor_msgs__msg__PointCloud2(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t max_serialized_size_key_sensor_msgs__msg__PointCloud2(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_lio_sam
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _CloudInfo__ros_msg_type = lio_sam__msg__CloudInfo;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
bool cdr_serialize_lio_sam__msg__CloudInfo(
  const lio_sam__msg__CloudInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: start_ring_index
  {
    size_t size = ros_message->start_ring_index.size;
    auto array_ptr = ros_message->start_ring_index.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: end_ring_index
  {
    size_t size = ros_message->end_ring_index.size;
    auto array_ptr = ros_message->end_ring_index.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: point_col_ind
  {
    size_t size = ros_message->point_col_ind.size;
    auto array_ptr = ros_message->point_col_ind.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: point_range
  {
    size_t size = ros_message->point_range.size;
    auto array_ptr = ros_message->point_range.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: imu_available
  {
    cdr << ros_message->imu_available;
  }

  // Field name: odom_available
  {
    cdr << ros_message->odom_available;
  }

  // Field name: imu_roll_init
  {
    cdr << ros_message->imu_roll_init;
  }

  // Field name: imu_pitch_init
  {
    cdr << ros_message->imu_pitch_init;
  }

  // Field name: imu_yaw_init
  {
    cdr << ros_message->imu_yaw_init;
  }

  // Field name: initial_guess_x
  {
    cdr << ros_message->initial_guess_x;
  }

  // Field name: initial_guess_y
  {
    cdr << ros_message->initial_guess_y;
  }

  // Field name: initial_guess_z
  {
    cdr << ros_message->initial_guess_z;
  }

  // Field name: initial_guess_roll
  {
    cdr << ros_message->initial_guess_roll;
  }

  // Field name: initial_guess_pitch
  {
    cdr << ros_message->initial_guess_pitch;
  }

  // Field name: initial_guess_yaw
  {
    cdr << ros_message->initial_guess_yaw;
  }

  // Field name: cloud_deskewed
  {
    cdr_serialize_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_deskewed, cdr);
  }

  // Field name: cloud_corner
  {
    cdr_serialize_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_corner, cdr);
  }

  // Field name: cloud_surface
  {
    cdr_serialize_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_surface, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
bool cdr_deserialize_lio_sam__msg__CloudInfo(
  eprosima::fastcdr::Cdr & cdr,
  lio_sam__msg__CloudInfo * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: start_ring_index
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->start_ring_index.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->start_ring_index);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->start_ring_index, size)) {
      fprintf(stderr, "failed to create array for field 'start_ring_index'");
      return false;
    }
    auto array_ptr = ros_message->start_ring_index.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: end_ring_index
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->end_ring_index.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->end_ring_index);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->end_ring_index, size)) {
      fprintf(stderr, "failed to create array for field 'end_ring_index'");
      return false;
    }
    auto array_ptr = ros_message->end_ring_index.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: point_col_ind
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->point_col_ind.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->point_col_ind);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->point_col_ind, size)) {
      fprintf(stderr, "failed to create array for field 'point_col_ind'");
      return false;
    }
    auto array_ptr = ros_message->point_col_ind.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: point_range
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->point_range.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->point_range);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->point_range, size)) {
      fprintf(stderr, "failed to create array for field 'point_range'");
      return false;
    }
    auto array_ptr = ros_message->point_range.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: imu_available
  {
    cdr >> ros_message->imu_available;
  }

  // Field name: odom_available
  {
    cdr >> ros_message->odom_available;
  }

  // Field name: imu_roll_init
  {
    cdr >> ros_message->imu_roll_init;
  }

  // Field name: imu_pitch_init
  {
    cdr >> ros_message->imu_pitch_init;
  }

  // Field name: imu_yaw_init
  {
    cdr >> ros_message->imu_yaw_init;
  }

  // Field name: initial_guess_x
  {
    cdr >> ros_message->initial_guess_x;
  }

  // Field name: initial_guess_y
  {
    cdr >> ros_message->initial_guess_y;
  }

  // Field name: initial_guess_z
  {
    cdr >> ros_message->initial_guess_z;
  }

  // Field name: initial_guess_roll
  {
    cdr >> ros_message->initial_guess_roll;
  }

  // Field name: initial_guess_pitch
  {
    cdr >> ros_message->initial_guess_pitch;
  }

  // Field name: initial_guess_yaw
  {
    cdr >> ros_message->initial_guess_yaw;
  }

  // Field name: cloud_deskewed
  {
    cdr_deserialize_sensor_msgs__msg__PointCloud2(cdr, &ros_message->cloud_deskewed);
  }

  // Field name: cloud_corner
  {
    cdr_deserialize_sensor_msgs__msg__PointCloud2(cdr, &ros_message->cloud_corner);
  }

  // Field name: cloud_surface
  {
    cdr_deserialize_sensor_msgs__msg__PointCloud2(cdr, &ros_message->cloud_surface);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
size_t get_serialized_size_lio_sam__msg__CloudInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CloudInfo__ros_msg_type * ros_message = static_cast<const _CloudInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: start_ring_index
  {
    size_t array_size = ros_message->start_ring_index.size;
    auto array_ptr = ros_message->start_ring_index.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: end_ring_index
  {
    size_t array_size = ros_message->end_ring_index.size;
    auto array_ptr = ros_message->end_ring_index.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: point_col_ind
  {
    size_t array_size = ros_message->point_col_ind.size;
    auto array_ptr = ros_message->point_col_ind.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: point_range
  {
    size_t array_size = ros_message->point_range.size;
    auto array_ptr = ros_message->point_range.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_available
  {
    size_t item_size = sizeof(ros_message->imu_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: odom_available
  {
    size_t item_size = sizeof(ros_message->odom_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_roll_init
  {
    size_t item_size = sizeof(ros_message->imu_roll_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_pitch_init
  {
    size_t item_size = sizeof(ros_message->imu_pitch_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_yaw_init
  {
    size_t item_size = sizeof(ros_message->imu_yaw_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_x
  {
    size_t item_size = sizeof(ros_message->initial_guess_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_y
  {
    size_t item_size = sizeof(ros_message->initial_guess_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_z
  {
    size_t item_size = sizeof(ros_message->initial_guess_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_roll
  {
    size_t item_size = sizeof(ros_message->initial_guess_roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_pitch
  {
    size_t item_size = sizeof(ros_message->initial_guess_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_yaw
  {
    size_t item_size = sizeof(ros_message->initial_guess_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cloud_deskewed
  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_deskewed), current_alignment);

  // Field name: cloud_corner
  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_corner), current_alignment);

  // Field name: cloud_surface
  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_surface), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
size_t max_serialized_size_lio_sam__msg__CloudInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: start_ring_index
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: end_ring_index
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: point_col_ind
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: point_range
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: imu_available
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: odom_available
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: imu_roll_init
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: imu_pitch_init
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: imu_yaw_init
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_z
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_roll
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: cloud_deskewed
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: cloud_corner
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: cloud_surface
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = lio_sam__msg__CloudInfo;
    is_plain =
      (
      offsetof(DataType, cloud_surface) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
bool cdr_serialize_key_lio_sam__msg__CloudInfo(
  const lio_sam__msg__CloudInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: start_ring_index
  {
    size_t size = ros_message->start_ring_index.size;
    auto array_ptr = ros_message->start_ring_index.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: end_ring_index
  {
    size_t size = ros_message->end_ring_index.size;
    auto array_ptr = ros_message->end_ring_index.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: point_col_ind
  {
    size_t size = ros_message->point_col_ind.size;
    auto array_ptr = ros_message->point_col_ind.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: point_range
  {
    size_t size = ros_message->point_range.size;
    auto array_ptr = ros_message->point_range.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: imu_available
  {
    cdr << ros_message->imu_available;
  }

  // Field name: odom_available
  {
    cdr << ros_message->odom_available;
  }

  // Field name: imu_roll_init
  {
    cdr << ros_message->imu_roll_init;
  }

  // Field name: imu_pitch_init
  {
    cdr << ros_message->imu_pitch_init;
  }

  // Field name: imu_yaw_init
  {
    cdr << ros_message->imu_yaw_init;
  }

  // Field name: initial_guess_x
  {
    cdr << ros_message->initial_guess_x;
  }

  // Field name: initial_guess_y
  {
    cdr << ros_message->initial_guess_y;
  }

  // Field name: initial_guess_z
  {
    cdr << ros_message->initial_guess_z;
  }

  // Field name: initial_guess_roll
  {
    cdr << ros_message->initial_guess_roll;
  }

  // Field name: initial_guess_pitch
  {
    cdr << ros_message->initial_guess_pitch;
  }

  // Field name: initial_guess_yaw
  {
    cdr << ros_message->initial_guess_yaw;
  }

  // Field name: cloud_deskewed
  {
    cdr_serialize_key_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_deskewed, cdr);
  }

  // Field name: cloud_corner
  {
    cdr_serialize_key_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_corner, cdr);
  }

  // Field name: cloud_surface
  {
    cdr_serialize_key_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_surface, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
size_t get_serialized_size_key_lio_sam__msg__CloudInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CloudInfo__ros_msg_type * ros_message = static_cast<const _CloudInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: start_ring_index
  {
    size_t array_size = ros_message->start_ring_index.size;
    auto array_ptr = ros_message->start_ring_index.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: end_ring_index
  {
    size_t array_size = ros_message->end_ring_index.size;
    auto array_ptr = ros_message->end_ring_index.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: point_col_ind
  {
    size_t array_size = ros_message->point_col_ind.size;
    auto array_ptr = ros_message->point_col_ind.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: point_range
  {
    size_t array_size = ros_message->point_range.size;
    auto array_ptr = ros_message->point_range.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_available
  {
    size_t item_size = sizeof(ros_message->imu_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: odom_available
  {
    size_t item_size = sizeof(ros_message->odom_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_roll_init
  {
    size_t item_size = sizeof(ros_message->imu_roll_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_pitch_init
  {
    size_t item_size = sizeof(ros_message->imu_pitch_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: imu_yaw_init
  {
    size_t item_size = sizeof(ros_message->imu_yaw_init);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_x
  {
    size_t item_size = sizeof(ros_message->initial_guess_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_y
  {
    size_t item_size = sizeof(ros_message->initial_guess_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_z
  {
    size_t item_size = sizeof(ros_message->initial_guess_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_roll
  {
    size_t item_size = sizeof(ros_message->initial_guess_roll);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_pitch
  {
    size_t item_size = sizeof(ros_message->initial_guess_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: initial_guess_yaw
  {
    size_t item_size = sizeof(ros_message->initial_guess_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cloud_deskewed
  current_alignment += get_serialized_size_key_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_deskewed), current_alignment);

  // Field name: cloud_corner
  current_alignment += get_serialized_size_key_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_corner), current_alignment);

  // Field name: cloud_surface
  current_alignment += get_serialized_size_key_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_surface), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_lio_sam
size_t max_serialized_size_key_lio_sam__msg__CloudInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: start_ring_index
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: end_ring_index
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: point_col_ind
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: point_range
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: imu_available
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: odom_available
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: imu_roll_init
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: imu_pitch_init
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: imu_yaw_init
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_x
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_y
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_z
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_roll
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_pitch
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: initial_guess_yaw
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: cloud_deskewed
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: cloud_corner
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: cloud_surface
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = lio_sam__msg__CloudInfo;
    is_plain =
      (
      offsetof(DataType, cloud_surface) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _CloudInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const lio_sam__msg__CloudInfo * ros_message = static_cast<const lio_sam__msg__CloudInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_lio_sam__msg__CloudInfo(ros_message, cdr);
}

static bool _CloudInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  lio_sam__msg__CloudInfo * ros_message = static_cast<lio_sam__msg__CloudInfo *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_lio_sam__msg__CloudInfo(cdr, ros_message);
}

static uint32_t _CloudInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_lio_sam__msg__CloudInfo(
      untyped_ros_message, 0));
}

static size_t _CloudInfo__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_lio_sam__msg__CloudInfo(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_CloudInfo = {
  "lio_sam::msg",
  "CloudInfo",
  _CloudInfo__cdr_serialize,
  _CloudInfo__cdr_deserialize,
  _CloudInfo__get_serialized_size,
  _CloudInfo__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _CloudInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CloudInfo,
  get_message_typesupport_handle_function,
  &lio_sam__msg__CloudInfo__get_type_hash,
  &lio_sam__msg__CloudInfo__get_type_description,
  &lio_sam__msg__CloudInfo__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, lio_sam, msg, CloudInfo)() {
  return &_CloudInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
