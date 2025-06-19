// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from cartographer_ros_msgs:msg/TrajectoryStates.idl
// generated code does not contain a copyright notice
#include "cartographer_ros_msgs/msg/detail/trajectory_states__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "cartographer_ros_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "cartographer_ros_msgs/msg/detail/trajectory_states__struct.h"
#include "cartographer_ros_msgs/msg/detail/trajectory_states__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // trajectory_id, trajectory_state
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // trajectory_id, trajectory_state
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_cartographer_ros_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _TrajectoryStates__ros_msg_type = cartographer_ros_msgs__msg__TrajectoryStates;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
bool cdr_serialize_cartographer_ros_msgs__msg__TrajectoryStates(
  const cartographer_ros_msgs__msg__TrajectoryStates * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: trajectory_id
  {
    size_t size = ros_message->trajectory_id.size;
    auto array_ptr = ros_message->trajectory_id.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: trajectory_state
  {
    size_t size = ros_message->trajectory_state.size;
    auto array_ptr = ros_message->trajectory_state.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
bool cdr_deserialize_cartographer_ros_msgs__msg__TrajectoryStates(
  eprosima::fastcdr::Cdr & cdr,
  cartographer_ros_msgs__msg__TrajectoryStates * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: trajectory_id
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->trajectory_id.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->trajectory_id);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->trajectory_id, size)) {
      fprintf(stderr, "failed to create array for field 'trajectory_id'");
      return false;
    }
    auto array_ptr = ros_message->trajectory_id.data;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: trajectory_state
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->trajectory_state.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->trajectory_state);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->trajectory_state, size)) {
      fprintf(stderr, "failed to create array for field 'trajectory_state'");
      return false;
    }
    auto array_ptr = ros_message->trajectory_state.data;
    cdr.deserialize_array(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t get_serialized_size_cartographer_ros_msgs__msg__TrajectoryStates(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TrajectoryStates__ros_msg_type * ros_message = static_cast<const _TrajectoryStates__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: trajectory_id
  {
    size_t array_size = ros_message->trajectory_id.size;
    auto array_ptr = ros_message->trajectory_id.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: trajectory_state
  {
    size_t array_size = ros_message->trajectory_state.size;
    auto array_ptr = ros_message->trajectory_state.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t max_serialized_size_cartographer_ros_msgs__msg__TrajectoryStates(
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

  // Field name: trajectory_id
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

  // Field name: trajectory_state
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cartographer_ros_msgs__msg__TrajectoryStates;
    is_plain =
      (
      offsetof(DataType, trajectory_state) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
bool cdr_serialize_key_cartographer_ros_msgs__msg__TrajectoryStates(
  const cartographer_ros_msgs__msg__TrajectoryStates * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: trajectory_id
  {
    size_t size = ros_message->trajectory_id.size;
    auto array_ptr = ros_message->trajectory_id.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: trajectory_state
  {
    size_t size = ros_message->trajectory_state.size;
    auto array_ptr = ros_message->trajectory_state.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serialize_array(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t get_serialized_size_key_cartographer_ros_msgs__msg__TrajectoryStates(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TrajectoryStates__ros_msg_type * ros_message = static_cast<const _TrajectoryStates__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: trajectory_id
  {
    size_t array_size = ros_message->trajectory_id.size;
    auto array_ptr = ros_message->trajectory_id.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: trajectory_state
  {
    size_t array_size = ros_message->trajectory_state.size;
    auto array_ptr = ros_message->trajectory_state.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_cartographer_ros_msgs
size_t max_serialized_size_key_cartographer_ros_msgs__msg__TrajectoryStates(
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

  // Field name: trajectory_id
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

  // Field name: trajectory_state
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = cartographer_ros_msgs__msg__TrajectoryStates;
    is_plain =
      (
      offsetof(DataType, trajectory_state) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _TrajectoryStates__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const cartographer_ros_msgs__msg__TrajectoryStates * ros_message = static_cast<const cartographer_ros_msgs__msg__TrajectoryStates *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_cartographer_ros_msgs__msg__TrajectoryStates(ros_message, cdr);
}

static bool _TrajectoryStates__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  cartographer_ros_msgs__msg__TrajectoryStates * ros_message = static_cast<cartographer_ros_msgs__msg__TrajectoryStates *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_cartographer_ros_msgs__msg__TrajectoryStates(cdr, ros_message);
}

static uint32_t _TrajectoryStates__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_cartographer_ros_msgs__msg__TrajectoryStates(
      untyped_ros_message, 0));
}

static size_t _TrajectoryStates__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_cartographer_ros_msgs__msg__TrajectoryStates(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TrajectoryStates = {
  "cartographer_ros_msgs::msg",
  "TrajectoryStates",
  _TrajectoryStates__cdr_serialize,
  _TrajectoryStates__cdr_deserialize,
  _TrajectoryStates__get_serialized_size,
  _TrajectoryStates__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TrajectoryStates__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TrajectoryStates,
  get_message_typesupport_handle_function,
  &cartographer_ros_msgs__msg__TrajectoryStates__get_type_hash,
  &cartographer_ros_msgs__msg__TrajectoryStates__get_type_description,
  &cartographer_ros_msgs__msg__TrajectoryStates__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cartographer_ros_msgs, msg, TrajectoryStates)() {
  return &_TrajectoryStates__type_support;
}

#if defined(__cplusplus)
}
#endif
