// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:srv/SubmapQuery.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/srv/detail/submap_query__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__SubmapQuery__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xf6, 0xde, 0xb8, 0xa5, 0xb0, 0xe1, 0xcb, 0xf2,
      0x2c, 0xda, 0x90, 0xc1, 0xed, 0xfd, 0xc4, 0x0c,
      0xde, 0x7e, 0x0f, 0x64, 0x17, 0xcf, 0x74, 0x47,
      0x8f, 0x10, 0x6f, 0xb4, 0x0c, 0x53, 0xa9, 0x07,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0xfb, 0x3e, 0x2c, 0xfd, 0xea, 0x63, 0x81,
      0x72, 0x32, 0xfb, 0xc1, 0x77, 0x0d, 0xd7, 0xa1,
      0xbe, 0xab, 0xb0, 0xad, 0x4a, 0xad, 0xdb, 0x54,
      0x4e, 0x82, 0x70, 0xc1, 0x89, 0x77, 0xcf, 0x37,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x7f, 0xac, 0xe9, 0x9f, 0xd7, 0xb0, 0x8b, 0x3b,
      0x7a, 0xbd, 0x57, 0x1c, 0x53, 0x4e, 0xe1, 0xd2,
      0x26, 0x5d, 0xd4, 0xaf, 0xeb, 0xee, 0x90, 0x92,
      0xf4, 0x68, 0x36, 0xf4, 0x74, 0x3b, 0xa7, 0xee,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xb5, 0x7c, 0x05, 0x10, 0xdb, 0xeb, 0x74, 0xf9,
      0xd4, 0xea, 0x4f, 0x8c, 0x63, 0x55, 0x88, 0x49,
      0x68, 0x19, 0x88, 0x2c, 0xda, 0x3c, 0x03, 0xcf,
      0x7e, 0x7e, 0xa8, 0x99, 0xd9, 0x4e, 0x37, 0x8a,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "cartographer_ros_msgs/msg/detail/submap_texture__functions.h"
#include "geometry_msgs/msg/detail/point__functions.h"
#include "cartographer_ros_msgs/msg/detail/status_response__functions.h"
#include "geometry_msgs/msg/detail/pose__functions.h"
#include "geometry_msgs/msg/detail/quaternion__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH = {1, {
    0x95, 0xaa, 0x98, 0xef, 0xf3, 0x25, 0xb4, 0x01,
    0xaf, 0x49, 0x25, 0x03, 0x96, 0xad, 0x39, 0x2d,
    0x30, 0x6c, 0xd5, 0x00, 0x18, 0x81, 0x85, 0x7a,
    0x01, 0x6e, 0x22, 0x94, 0x68, 0x77, 0x8f, 0x36,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__SubmapTexture__EXPECTED_HASH = {1, {
    0x04, 0x22, 0xb9, 0xde, 0x7e, 0x92, 0xbb, 0x16,
    0xdf, 0x61, 0x76, 0x0d, 0xe2, 0x83, 0x4d, 0x06,
    0x6d, 0x62, 0x1d, 0x16, 0x2d, 0x33, 0x46, 0x46,
    0x40, 0x35, 0x8e, 0x5b, 0x10, 0xf1, 0xef, 0xc8,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Point__EXPECTED_HASH = {1, {
    0x69, 0x63, 0x08, 0x48, 0x42, 0xa9, 0xb0, 0x44,
    0x94, 0xd6, 0xb2, 0x94, 0x1d, 0x11, 0x44, 0x47,
    0x08, 0xd8, 0x92, 0xda, 0x2f, 0x4b, 0x09, 0x84,
    0x3b, 0x9c, 0x43, 0xf4, 0x2a, 0x7f, 0x68, 0x81,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Pose__EXPECTED_HASH = {1, {
    0xd5, 0x01, 0x95, 0x4e, 0x94, 0x76, 0xce, 0xa2,
    0x99, 0x69, 0x84, 0xe8, 0x12, 0x05, 0x4b, 0x68,
    0x02, 0x6a, 0xe0, 0xbf, 0xae, 0x78, 0x9d, 0x9a,
    0x10, 0xb2, 0x3d, 0xaf, 0x35, 0xcc, 0x90, 0xfa,
  }};
static const rosidl_type_hash_t geometry_msgs__msg__Quaternion__EXPECTED_HASH = {1, {
    0x8a, 0x76, 0x5f, 0x66, 0x77, 0x8c, 0x8f, 0xf7,
    0xc8, 0xab, 0x94, 0xaf, 0xcc, 0x59, 0x0a, 0x2e,
    0xd5, 0x32, 0x5a, 0x1d, 0x9a, 0x07, 0x6f, 0xff,
    0xf3, 0x8f, 0xbc, 0xe3, 0x6f, 0x45, 0x86, 0x84,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char cartographer_ros_msgs__srv__SubmapQuery__TYPE_NAME[] = "cartographer_ros_msgs/srv/SubmapQuery";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME[] = "cartographer_ros_msgs/msg/StatusResponse";
static char cartographer_ros_msgs__msg__SubmapTexture__TYPE_NAME[] = "cartographer_ros_msgs/msg/SubmapTexture";
static char cartographer_ros_msgs__srv__SubmapQuery_Event__TYPE_NAME[] = "cartographer_ros_msgs/srv/SubmapQuery_Event";
static char cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME[] = "cartographer_ros_msgs/srv/SubmapQuery_Request";
static char cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME[] = "cartographer_ros_msgs/srv/SubmapQuery_Response";
static char geometry_msgs__msg__Point__TYPE_NAME[] = "geometry_msgs/msg/Point";
static char geometry_msgs__msg__Pose__TYPE_NAME[] = "geometry_msgs/msg/Pose";
static char geometry_msgs__msg__Quaternion__TYPE_NAME[] = "geometry_msgs/msg/Quaternion";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__SubmapQuery__FIELD_NAME__request_message[] = "request_message";
static char cartographer_ros_msgs__srv__SubmapQuery__FIELD_NAME__response_message[] = "response_message";
static char cartographer_ros_msgs__srv__SubmapQuery__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__SubmapQuery__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__SubmapQuery__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__SubmapQuery_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__SubmapQuery__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__SubmapTexture__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__SubmapQuery__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__SubmapQuery__TYPE_NAME, 37, 37},
      {cartographer_ros_msgs__srv__SubmapQuery__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__SubmapQuery__REFERENCED_TYPE_DESCRIPTIONS, 10, 10},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__SubmapTexture__EXPECTED_HASH, cartographer_ros_msgs__msg__SubmapTexture__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__SubmapTexture__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__SubmapQuery_Request__FIELD_NAME__trajectory_id[] = "trajectory_id";
static char cartographer_ros_msgs__srv__SubmapQuery_Request__FIELD_NAME__submap_index[] = "submap_index";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__SubmapQuery_Request__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Request__FIELD_NAME__trajectory_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Request__FIELD_NAME__submap_index, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME, 45, 45},
      {cartographer_ros_msgs__srv__SubmapQuery_Request__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__SubmapQuery_Response__FIELD_NAME__status[] = "status";
static char cartographer_ros_msgs__srv__SubmapQuery_Response__FIELD_NAME__submap_version[] = "submap_version";
static char cartographer_ros_msgs__srv__SubmapQuery_Response__FIELD_NAME__textures[] = "textures";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__SubmapQuery_Response__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Response__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Response__FIELD_NAME__submap_version, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Response__FIELD_NAME__textures, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {cartographer_ros_msgs__msg__SubmapTexture__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__SubmapQuery_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__SubmapTexture__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME, 46, 46},
      {cartographer_ros_msgs__srv__SubmapQuery_Response__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__SubmapQuery_Response__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__SubmapTexture__EXPECTED_HASH, cartographer_ros_msgs__msg__SubmapTexture__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__SubmapTexture__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__SubmapQuery_Event__FIELD_NAME__info[] = "info";
static char cartographer_ros_msgs__srv__SubmapQuery_Event__FIELD_NAME__request[] = "request";
static char cartographer_ros_msgs__srv__SubmapQuery_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__SubmapQuery_Event__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__SubmapQuery_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__SubmapTexture__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Point__TYPE_NAME, 23, 23},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Pose__TYPE_NAME, 22, 22},
    {NULL, 0, 0},
  },
  {
    {geometry_msgs__msg__Quaternion__TYPE_NAME, 28, 28},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__SubmapQuery_Event__TYPE_NAME, 43, 43},
      {cartographer_ros_msgs__srv__SubmapQuery_Event__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__SubmapQuery_Event__REFERENCED_TYPE_DESCRIPTIONS, 9, 9},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__SubmapTexture__EXPECTED_HASH, cartographer_ros_msgs__msg__SubmapTexture__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__SubmapTexture__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Point__EXPECTED_HASH, geometry_msgs__msg__Point__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = geometry_msgs__msg__Point__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Pose__EXPECTED_HASH, geometry_msgs__msg__Pose__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = geometry_msgs__msg__Pose__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&geometry_msgs__msg__Quaternion__EXPECTED_HASH, geometry_msgs__msg__Quaternion__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = geometry_msgs__msg__Quaternion__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Copyright 2016 The Cartographer Authors\n"
  "#\n"
  "# Licensed under the Apache License, Version 2.0 (the \"License\");\n"
  "# you may not use this file except in compliance with the License.\n"
  "# You may obtain a copy of the License at\n"
  "#\n"
  "#      http://www.apache.org/licenses/LICENSE-2.0\n"
  "#\n"
  "# Unless required by applicable law or agreed to in writing, software\n"
  "# distributed under the License is distributed on an \"AS IS\" BASIS,\n"
  "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
  "# See the License for the specific language governing permissions and\n"
  "# limitations under the License.\n"
  "\n"
  "int32 trajectory_id\n"
  "int32 submap_index\n"
  "---\n"
  "cartographer_ros_msgs/StatusResponse status\n"
  "int32 submap_version\n"
  "cartographer_ros_msgs/SubmapTexture[] textures";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__SubmapQuery__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__SubmapQuery__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 745, 745},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__SubmapQuery_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__SubmapQuery_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__SubmapQuery_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__SubmapQuery_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__SubmapQuery_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__SubmapQuery_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__SubmapQuery__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[11];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 11, 11};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__SubmapQuery__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__SubmapTexture__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__srv__SubmapQuery_Event__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__srv__SubmapQuery_Request__get_individual_type_description_source(NULL);
    sources[6] = *cartographer_ros_msgs__srv__SubmapQuery_Response__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[9] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[10] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__SubmapQuery_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__SubmapQuery_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__SubmapQuery_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__SubmapQuery_Response__get_individual_type_description_source(NULL),
    sources[1] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__SubmapTexture__get_individual_type_description_source(NULL);
    sources[3] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[4] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[5] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__SubmapQuery_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[10];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 10, 10};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__SubmapQuery_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__SubmapTexture__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__srv__SubmapQuery_Request__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__srv__SubmapQuery_Response__get_individual_type_description_source(NULL);
    sources[6] = *geometry_msgs__msg__Point__get_individual_type_description_source(NULL);
    sources[7] = *geometry_msgs__msg__Pose__get_individual_type_description_source(NULL);
    sources[8] = *geometry_msgs__msg__Quaternion__get_individual_type_description_source(NULL);
    sources[9] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
