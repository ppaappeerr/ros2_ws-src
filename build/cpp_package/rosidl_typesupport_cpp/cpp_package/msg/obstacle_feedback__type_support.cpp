// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "cpp_package/msg/detail/obstacle_feedback__functions.h"
#include "cpp_package/msg/detail/obstacle_feedback__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace cpp_package
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _ObstacleFeedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ObstacleFeedback_type_support_ids_t;

static const _ObstacleFeedback_type_support_ids_t _ObstacleFeedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ObstacleFeedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ObstacleFeedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ObstacleFeedback_type_support_symbol_names_t _ObstacleFeedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, cpp_package, msg, ObstacleFeedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, cpp_package, msg, ObstacleFeedback)),
  }
};

typedef struct _ObstacleFeedback_type_support_data_t
{
  void * data[2];
} _ObstacleFeedback_type_support_data_t;

static _ObstacleFeedback_type_support_data_t _ObstacleFeedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ObstacleFeedback_message_typesupport_map = {
  2,
  "cpp_package",
  &_ObstacleFeedback_message_typesupport_ids.typesupport_identifier[0],
  &_ObstacleFeedback_message_typesupport_symbol_names.symbol_name[0],
  &_ObstacleFeedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ObstacleFeedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ObstacleFeedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &cpp_package__msg__ObstacleFeedback__get_type_hash,
  &cpp_package__msg__ObstacleFeedback__get_type_description,
  &cpp_package__msg__ObstacleFeedback__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace cpp_package

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<cpp_package::msg::ObstacleFeedback>()
{
  return &::cpp_package::msg::rosidl_typesupport_cpp::ObstacleFeedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, cpp_package, msg, ObstacleFeedback)() {
  return get_message_type_support_handle<cpp_package::msg::ObstacleFeedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
