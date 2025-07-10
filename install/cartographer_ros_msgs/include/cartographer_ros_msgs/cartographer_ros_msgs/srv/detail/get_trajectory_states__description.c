// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:srv/GetTrajectoryStates.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/srv/detail/get_trajectory_states__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__GetTrajectoryStates__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0x83, 0x86, 0xb4, 0x65, 0xd0, 0x4c, 0xe9,
      0x83, 0x65, 0xf1, 0x8b, 0xb5, 0xc2, 0x7a, 0x34,
      0x8c, 0x06, 0x2c, 0xac, 0x64, 0x5a, 0x66, 0x7d,
      0x2b, 0xdc, 0x3e, 0xc4, 0xbb, 0xfa, 0xdf, 0x00,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5f, 0xd3, 0x1a, 0x3c, 0xe8, 0xe3, 0x20, 0xe8,
      0x84, 0x23, 0x38, 0x0c, 0x9f, 0x3d, 0xe3, 0x47,
      0xc9, 0x51, 0x7d, 0x9e, 0x4e, 0x60, 0x18, 0x22,
      0xbe, 0xce, 0x1f, 0xaf, 0x75, 0xfe, 0x88, 0x27,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x28, 0xfb, 0xde, 0xcf, 0x59, 0x79, 0xba, 0xf7,
      0xa9, 0xdc, 0xb2, 0xd1, 0x46, 0xc9, 0xd5, 0x84,
      0x59, 0x12, 0x3b, 0x7d, 0x6e, 0xb4, 0x7f, 0xb0,
      0x35, 0x26, 0x4c, 0x20, 0xbb, 0x78, 0x89, 0x86,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x06, 0xa0, 0x28, 0x96, 0x62, 0x46, 0x27, 0x18,
      0xaf, 0x9b, 0x34, 0x3a, 0x31, 0x8a, 0x55, 0xf3,
      0x2a, 0x86, 0xbb, 0x5d, 0x56, 0x63, 0x32, 0xc4,
      0xa2, 0x08, 0x44, 0x15, 0xc8, 0x92, 0x81, 0xb2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "cartographer_ros_msgs/msg/detail/status_response__functions.h"
#include "cartographer_ros_msgs/msg/detail/trajectory_states__functions.h"
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
static const rosidl_type_hash_t cartographer_ros_msgs__msg__TrajectoryStates__EXPECTED_HASH = {1, {
    0x11, 0x0c, 0x24, 0xec, 0xf1, 0x3f, 0x1f, 0xa1,
    0x44, 0xb1, 0x77, 0xa5, 0x5d, 0x56, 0xae, 0x67,
    0x69, 0x24, 0x02, 0xc4, 0x84, 0xae, 0x6b, 0x5f,
    0xe1, 0x99, 0x52, 0x53, 0xd8, 0xa7, 0x9c, 0x39,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char cartographer_ros_msgs__srv__GetTrajectoryStates__TYPE_NAME[] = "cartographer_ros_msgs/srv/GetTrajectoryStates";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME[] = "cartographer_ros_msgs/msg/StatusResponse";
static char cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME[] = "cartographer_ros_msgs/msg/TrajectoryStates";
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Event__TYPE_NAME[] = "cartographer_ros_msgs/srv/GetTrajectoryStates_Event";
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME[] = "cartographer_ros_msgs/srv/GetTrajectoryStates_Request";
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME[] = "cartographer_ros_msgs/srv/GetTrajectoryStates_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__GetTrajectoryStates__FIELD_NAME__request_message[] = "request_message";
static char cartographer_ros_msgs__srv__GetTrajectoryStates__FIELD_NAME__response_message[] = "response_message";
static char cartographer_ros_msgs__srv__GetTrajectoryStates__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__GetTrajectoryStates__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__GetTrajectoryStates__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__GetTrajectoryStates__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__GetTrajectoryStates__TYPE_NAME, 45, 45},
      {cartographer_ros_msgs__srv__GetTrajectoryStates__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__GetTrajectoryStates__REFERENCED_TYPE_DESCRIPTIONS, 8, 8},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__TrajectoryStates__EXPECTED_HASH, cartographer_ros_msgs__msg__TrajectoryStates__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__TrajectoryStates__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[5].fields = cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[7].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__GetTrajectoryStates_Request__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME, 53, 53},
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Response__FIELD_NAME__status[] = "status";
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Response__FIELD_NAME__trajectory_states[] = "trajectory_states";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__GetTrajectoryStates_Response__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__FIELD_NAME__trajectory_states, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__GetTrajectoryStates_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME, 54, 54},
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__FIELDS, 2, 2},
    },
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__TrajectoryStates__EXPECTED_HASH, cartographer_ros_msgs__msg__TrajectoryStates__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__TrajectoryStates__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELD_NAME__info[] = "info";
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELD_NAME__request[] = "request";
static char cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME, 53, 53},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME, 54, 54},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__GetTrajectoryStates_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME, 53, 53},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME, 54, 54},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__TYPE_NAME, 51, 51},
      {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__REFERENCED_TYPE_DESCRIPTIONS, 7, 7},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__TrajectoryStates__EXPECTED_HASH, cartographer_ros_msgs__msg__TrajectoryStates__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__TrajectoryStates__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[6].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Copyright 2018 The Cartographer Authors\n"
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
  "\n"
  "---\n"
  "cartographer_ros_msgs/StatusResponse status\n"
  "cartographer_ros_msgs/TrajectoryStates trajectory_states";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__GetTrajectoryStates__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__GetTrajectoryStates__TYPE_NAME, 45, 45},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 696, 696},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Request__TYPE_NAME, 53, 53},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Response__TYPE_NAME, 54, 54},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__GetTrajectoryStates_Event__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__GetTrajectoryStates__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[9];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 9, 9};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__GetTrajectoryStates__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__TrajectoryStates__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_individual_type_description_source(NULL);
    sources[6] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_individual_type_description_source(NULL);
    sources[7] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[8] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__TrajectoryStates__get_individual_type_description_source(NULL);
    sources[4] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[8];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 8, 8};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__TrajectoryStates__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Request__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__srv__GetTrajectoryStates_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    sources[7] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
