// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from cartographer_ros_msgs:srv/SubmapQuery.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "cartographer_ros_msgs/srv/detail/submap_query__struct.h"
#include "cartographer_ros_msgs/srv/detail/submap_query__type_support.h"
#include "cartographer_ros_msgs/srv/detail/submap_query__functions.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _SubmapQuery_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SubmapQuery_Request_type_support_ids_t;

static const _SubmapQuery_Request_type_support_ids_t _SubmapQuery_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SubmapQuery_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SubmapQuery_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SubmapQuery_Request_type_support_symbol_names_t _SubmapQuery_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cartographer_ros_msgs, srv, SubmapQuery_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cartographer_ros_msgs, srv, SubmapQuery_Request)),
  }
};

typedef struct _SubmapQuery_Request_type_support_data_t
{
  void * data[2];
} _SubmapQuery_Request_type_support_data_t;

static _SubmapQuery_Request_type_support_data_t _SubmapQuery_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SubmapQuery_Request_message_typesupport_map = {
  2,
  "cartographer_ros_msgs",
  &_SubmapQuery_Request_message_typesupport_ids.typesupport_identifier[0],
  &_SubmapQuery_Request_message_typesupport_symbol_names.symbol_name[0],
  &_SubmapQuery_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SubmapQuery_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SubmapQuery_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_hash,
  &cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_description,
  &cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace cartographer_ros_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, cartographer_ros_msgs, srv, SubmapQuery_Request)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_c::SubmapQuery_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__struct.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__type_support.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _SubmapQuery_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SubmapQuery_Response_type_support_ids_t;

static const _SubmapQuery_Response_type_support_ids_t _SubmapQuery_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SubmapQuery_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SubmapQuery_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SubmapQuery_Response_type_support_symbol_names_t _SubmapQuery_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cartographer_ros_msgs, srv, SubmapQuery_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cartographer_ros_msgs, srv, SubmapQuery_Response)),
  }
};

typedef struct _SubmapQuery_Response_type_support_data_t
{
  void * data[2];
} _SubmapQuery_Response_type_support_data_t;

static _SubmapQuery_Response_type_support_data_t _SubmapQuery_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SubmapQuery_Response_message_typesupport_map = {
  2,
  "cartographer_ros_msgs",
  &_SubmapQuery_Response_message_typesupport_ids.typesupport_identifier[0],
  &_SubmapQuery_Response_message_typesupport_symbol_names.symbol_name[0],
  &_SubmapQuery_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SubmapQuery_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SubmapQuery_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_hash,
  &cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_description,
  &cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace cartographer_ros_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, cartographer_ros_msgs, srv, SubmapQuery_Response)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_c::SubmapQuery_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__struct.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__type_support.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__functions.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _SubmapQuery_Event_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SubmapQuery_Event_type_support_ids_t;

static const _SubmapQuery_Event_type_support_ids_t _SubmapQuery_Event_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SubmapQuery_Event_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SubmapQuery_Event_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SubmapQuery_Event_type_support_symbol_names_t _SubmapQuery_Event_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cartographer_ros_msgs, srv, SubmapQuery_Event)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cartographer_ros_msgs, srv, SubmapQuery_Event)),
  }
};

typedef struct _SubmapQuery_Event_type_support_data_t
{
  void * data[2];
} _SubmapQuery_Event_type_support_data_t;

static _SubmapQuery_Event_type_support_data_t _SubmapQuery_Event_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SubmapQuery_Event_message_typesupport_map = {
  2,
  "cartographer_ros_msgs",
  &_SubmapQuery_Event_message_typesupport_ids.typesupport_identifier[0],
  &_SubmapQuery_Event_message_typesupport_symbol_names.symbol_name[0],
  &_SubmapQuery_Event_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SubmapQuery_Event_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SubmapQuery_Event_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
  &cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_hash,
  &cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_description,
  &cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace cartographer_ros_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, cartographer_ros_msgs, srv, SubmapQuery_Event)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_c::SubmapQuery_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "cartographer_ros_msgs/srv/detail/submap_query__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
#include "service_msgs/msg/service_event_info.h"
#include "builtin_interfaces/msg/time.h"

namespace cartographer_ros_msgs
{

namespace srv
{

namespace rosidl_typesupport_c
{
typedef struct _SubmapQuery_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SubmapQuery_type_support_ids_t;

static const _SubmapQuery_type_support_ids_t _SubmapQuery_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _SubmapQuery_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SubmapQuery_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SubmapQuery_type_support_symbol_names_t _SubmapQuery_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, cartographer_ros_msgs, srv, SubmapQuery)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, cartographer_ros_msgs, srv, SubmapQuery)),
  }
};

typedef struct _SubmapQuery_type_support_data_t
{
  void * data[2];
} _SubmapQuery_type_support_data_t;

static _SubmapQuery_type_support_data_t _SubmapQuery_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SubmapQuery_service_typesupport_map = {
  2,
  "cartographer_ros_msgs",
  &_SubmapQuery_service_typesupport_ids.typesupport_identifier[0],
  &_SubmapQuery_service_typesupport_symbol_names.symbol_name[0],
  &_SubmapQuery_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t SubmapQuery_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SubmapQuery_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
  &SubmapQuery_Request_message_type_support_handle,
  &SubmapQuery_Response_message_type_support_handle,
  &SubmapQuery_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    cartographer_ros_msgs,
    srv,
    SubmapQuery
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    cartographer_ros_msgs,
    srv,
    SubmapQuery
  ),
  &cartographer_ros_msgs__srv__SubmapQuery__get_type_hash,
  &cartographer_ros_msgs__srv__SubmapQuery__get_type_description,
  &cartographer_ros_msgs__srv__SubmapQuery__get_type_description_sources,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace cartographer_ros_msgs

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, cartographer_ros_msgs, srv, SubmapQuery)() {
  return &::cartographer_ros_msgs::srv::rosidl_typesupport_c::SubmapQuery_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
