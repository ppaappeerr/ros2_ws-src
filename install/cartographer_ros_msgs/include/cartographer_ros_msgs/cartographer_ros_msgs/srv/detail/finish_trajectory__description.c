// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:srv/FinishTrajectory.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/srv/detail/finish_trajectory__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__FinishTrajectory__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x38, 0xd8, 0x56, 0xce, 0x60, 0xb8, 0x5a, 0x7d,
      0xa0, 0xf3, 0x88, 0x45, 0xd9, 0x31, 0xf3, 0x59,
      0x92, 0x1b, 0xd8, 0xb5, 0xaa, 0xfc, 0x57, 0x9e,
      0x0a, 0xc9, 0xcd, 0xf0, 0x14, 0xfb, 0xfd, 0x09,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x70, 0x20, 0x0c, 0xd7, 0xf9, 0xee, 0x01, 0xae,
      0xfd, 0x19, 0x6c, 0xf1, 0x4e, 0x3b, 0x81, 0x85,
      0xf5, 0x6d, 0xa9, 0x70, 0x24, 0x9d, 0x8d, 0xae,
      0x87, 0x6b, 0x04, 0x32, 0x11, 0xa7, 0x8e, 0x2c,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x95, 0xf0, 0x51, 0xd2, 0xb9, 0xc4, 0xac, 0x5e,
      0xcc, 0x96, 0x23, 0xf3, 0xce, 0xfd, 0xab, 0x32,
      0x53, 0x7b, 0x27, 0x0b, 0x8f, 0x9f, 0x4a, 0x58,
      0xbb, 0xb8, 0xbe, 0x01, 0x15, 0x6b, 0xd1, 0x00,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x26, 0xda, 0x9b, 0xc7, 0x2b, 0x99, 0xc2, 0x55,
      0xfd, 0x70, 0x29, 0x01, 0x58, 0xa1, 0xf4, 0x8d,
      0x5d, 0xc2, 0xb5, 0xe9, 0xea, 0x3b, 0xbf, 0xed,
      0x29, 0xda, 0x1f, 0x3c, 0xd7, 0x66, 0xea, 0x77,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "cartographer_ros_msgs/msg/detail/status_response__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"
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
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char cartographer_ros_msgs__srv__FinishTrajectory__TYPE_NAME[] = "cartographer_ros_msgs/srv/FinishTrajectory";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME[] = "cartographer_ros_msgs/msg/StatusResponse";
static char cartographer_ros_msgs__srv__FinishTrajectory_Event__TYPE_NAME[] = "cartographer_ros_msgs/srv/FinishTrajectory_Event";
static char cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME[] = "cartographer_ros_msgs/srv/FinishTrajectory_Request";
static char cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME[] = "cartographer_ros_msgs/srv/FinishTrajectory_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__FinishTrajectory__FIELD_NAME__request_message[] = "request_message";
static char cartographer_ros_msgs__srv__FinishTrajectory__FIELD_NAME__response_message[] = "response_message";
static char cartographer_ros_msgs__srv__FinishTrajectory__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__FinishTrajectory__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__FinishTrajectory__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__FinishTrajectory_Event__TYPE_NAME, 48, 48},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__FinishTrajectory__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Event__TYPE_NAME, 48, 48},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__FinishTrajectory__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__FinishTrajectory__TYPE_NAME, 42, 42},
      {cartographer_ros_msgs__srv__FinishTrajectory__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__FinishTrajectory__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__FinishTrajectory_Request__FIELD_NAME__trajectory_id[] = "trajectory_id";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__FinishTrajectory_Request__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Request__FIELD_NAME__trajectory_id, 13, 13},
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
cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME, 50, 50},
      {cartographer_ros_msgs__srv__FinishTrajectory_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__FinishTrajectory_Response__FIELD_NAME__status[] = "status";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__FinishTrajectory_Response__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Response__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__FinishTrajectory_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME, 51, 51},
      {cartographer_ros_msgs__srv__FinishTrajectory_Response__FIELDS, 1, 1},
    },
    {cartographer_ros_msgs__srv__FinishTrajectory_Response__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELD_NAME__info[] = "info";
static char cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELD_NAME__request[] = "request";
static char cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME, 50, 50},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__FinishTrajectory_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME, 50, 50},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__FinishTrajectory_Event__TYPE_NAME, 48, 48},
      {cartographer_ros_msgs__srv__FinishTrajectory_Event__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__FinishTrajectory_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
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
  "---\n"
  "cartographer_ros_msgs/StatusResponse status";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__FinishTrajectory__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__FinishTrajectory__TYPE_NAME, 42, 42},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 658, 658},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__FinishTrajectory_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__FinishTrajectory_Request__TYPE_NAME, 50, 50},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__FinishTrajectory_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__FinishTrajectory_Response__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__FinishTrajectory_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__FinishTrajectory_Event__TYPE_NAME, 48, 48},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__FinishTrajectory__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__FinishTrajectory__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__srv__FinishTrajectory_Event__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__srv__FinishTrajectory_Request__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__srv__FinishTrajectory_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__FinishTrajectory_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__FinishTrajectory_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__FinishTrajectory_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__FinishTrajectory_Response__get_individual_type_description_source(NULL),
    sources[1] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__FinishTrajectory_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__FinishTrajectory_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__srv__FinishTrajectory_Request__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__srv__FinishTrajectory_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
