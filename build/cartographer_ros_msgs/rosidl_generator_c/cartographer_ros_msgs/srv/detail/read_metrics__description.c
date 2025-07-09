// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:srv/ReadMetrics.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/srv/detail/read_metrics__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__ReadMetrics__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0x2d, 0x11, 0xe3, 0x9b, 0xb2, 0xf9, 0xaa,
      0x43, 0x60, 0x75, 0x91, 0x6b, 0xfa, 0x90, 0x51,
      0xbb, 0xb4, 0x72, 0x20, 0x57, 0x0c, 0xfe, 0x57,
      0xf1, 0x69, 0xea, 0x95, 0xab, 0xbb, 0xf0, 0x6b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1c, 0x16, 0xdc, 0xc7, 0x12, 0x84, 0x8f, 0xe3,
      0xe5, 0x05, 0xfe, 0x29, 0x9f, 0x22, 0xaa, 0xfd,
      0x62, 0x7c, 0x0f, 0x72, 0x0c, 0x8f, 0x92, 0xf5,
      0x90, 0xf1, 0xa1, 0xb0, 0xaa, 0x14, 0xb9, 0x65,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x40, 0x37, 0x61, 0x0f, 0xde, 0x86, 0x41, 0x84,
      0x63, 0x63, 0xc0, 0xb1, 0x80, 0x68, 0x07, 0x51,
      0xd2, 0x9d, 0x3a, 0xbd, 0xd1, 0xb2, 0x5a, 0x61,
      0xd4, 0xb5, 0x98, 0x32, 0xd8, 0x8e, 0x3a, 0xf7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x00, 0xf9, 0x4b, 0xe4, 0x41, 0x2f, 0xf2, 0x58,
      0xae, 0xb9, 0x32, 0x92, 0x45, 0xb5, 0x69, 0xaf,
      0xaf, 0x6b, 0xff, 0x47, 0xf3, 0x0f, 0x68, 0xc5,
      0x52, 0xf9, 0x7d, 0xfa, 0x70, 0x3c, 0xa4, 0x54,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "cartographer_ros_msgs/msg/detail/metric__functions.h"
#include "cartographer_ros_msgs/msg/detail/metric_family__functions.h"
#include "cartographer_ros_msgs/msg/detail/metric_label__functions.h"
#include "cartographer_ros_msgs/msg/detail/status_response__functions.h"
#include "cartographer_ros_msgs/msg/detail/histogram_bucket__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH = {1, {
    0x3a, 0x4a, 0xb1, 0x33, 0x65, 0xf7, 0x0f, 0x3f,
    0x9d, 0xe5, 0x0a, 0x4c, 0x41, 0x39, 0x8a, 0xb5,
    0xe1, 0x0a, 0xa9, 0xf8, 0x34, 0xf8, 0x05, 0x03,
    0xc8, 0x56, 0x80, 0x35, 0xa3, 0x30, 0xb8, 0x6b,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__Metric__EXPECTED_HASH = {1, {
    0xa2, 0xe4, 0x53, 0x2a, 0xe6, 0x41, 0x45, 0x63,
    0x3f, 0x94, 0x4a, 0xc4, 0xde, 0x6b, 0x49, 0x65,
    0x66, 0xc2, 0x03, 0x22, 0xfe, 0xd7, 0xe4, 0x24,
    0x7a, 0x60, 0xbd, 0xca, 0x16, 0x95, 0x2a, 0x60,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__MetricFamily__EXPECTED_HASH = {1, {
    0x3b, 0x06, 0x55, 0x16, 0xbc, 0x1d, 0x35, 0x21,
    0x18, 0x30, 0x81, 0x1e, 0x9d, 0xce, 0x7f, 0xbd,
    0xe2, 0xf8, 0x54, 0xcd, 0xd4, 0x31, 0x2f, 0x9c,
    0x0a, 0xa4, 0x28, 0xb1, 0xf5, 0x71, 0x18, 0xc6,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH = {1, {
    0x8b, 0xdd, 0x8b, 0x3d, 0x78, 0x1c, 0x0f, 0xcc,
    0xb4, 0x07, 0x84, 0x0d, 0x6e, 0x7a, 0x9c, 0x49,
    0x61, 0xf5, 0x1f, 0x3d, 0xcd, 0x15, 0x18, 0x16,
    0x52, 0x24, 0xbb, 0xa1, 0x70, 0xb1, 0xdb, 0x3f,
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

static char cartographer_ros_msgs__srv__ReadMetrics__TYPE_NAME[] = "cartographer_ros_msgs/srv/ReadMetrics";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME[] = "cartographer_ros_msgs/msg/HistogramBucket";
static char cartographer_ros_msgs__msg__Metric__TYPE_NAME[] = "cartographer_ros_msgs/msg/Metric";
static char cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME[] = "cartographer_ros_msgs/msg/MetricFamily";
static char cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME[] = "cartographer_ros_msgs/msg/MetricLabel";
static char cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME[] = "cartographer_ros_msgs/msg/StatusResponse";
static char cartographer_ros_msgs__srv__ReadMetrics_Event__TYPE_NAME[] = "cartographer_ros_msgs/srv/ReadMetrics_Event";
static char cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME[] = "cartographer_ros_msgs/srv/ReadMetrics_Request";
static char cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME[] = "cartographer_ros_msgs/srv/ReadMetrics_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__ReadMetrics__FIELD_NAME__request_message[] = "request_message";
static char cartographer_ros_msgs__srv__ReadMetrics__FIELD_NAME__response_message[] = "response_message";
static char cartographer_ros_msgs__srv__ReadMetrics__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__ReadMetrics__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__ReadMetrics__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__srv__ReadMetrics_Event__TYPE_NAME, 43, 43},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__ReadMetrics__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Event__TYPE_NAME, 43, 43},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__ReadMetrics__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__ReadMetrics__TYPE_NAME, 37, 37},
      {cartographer_ros_msgs__srv__ReadMetrics__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__ReadMetrics__REFERENCED_TYPE_DESCRIPTIONS, 10, 10},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH, cartographer_ros_msgs__msg__HistogramBucket__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__HistogramBucket__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__Metric__EXPECTED_HASH, cartographer_ros_msgs__msg__Metric__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__Metric__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricFamily__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricFamily__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__msg__MetricFamily__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricLabel__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__msg__MetricLabel__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[7].fields = cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[8].fields = cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[9].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__ReadMetrics_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__ReadMetrics_Request__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME, 45, 45},
      {cartographer_ros_msgs__srv__ReadMetrics_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__ReadMetrics_Response__FIELD_NAME__status[] = "status";
static char cartographer_ros_msgs__srv__ReadMetrics_Response__FIELD_NAME__metric_families[] = "metric_families";
static char cartographer_ros_msgs__srv__ReadMetrics_Response__FIELD_NAME__timestamp[] = "timestamp";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__ReadMetrics_Response__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Response__FIELD_NAME__status, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Response__FIELD_NAME__metric_families, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Response__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__ReadMetrics_Response__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME, 46, 46},
      {cartographer_ros_msgs__srv__ReadMetrics_Response__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__ReadMetrics_Response__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH, cartographer_ros_msgs__msg__HistogramBucket__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__HistogramBucket__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__Metric__EXPECTED_HASH, cartographer_ros_msgs__msg__Metric__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__Metric__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricFamily__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricFamily__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__msg__MetricFamily__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricLabel__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__msg__MetricLabel__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char cartographer_ros_msgs__srv__ReadMetrics_Event__FIELD_NAME__info[] = "info";
static char cartographer_ros_msgs__srv__ReadMetrics_Event__FIELD_NAME__request[] = "request";
static char cartographer_ros_msgs__srv__ReadMetrics_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__srv__ReadMetrics_Event__FIELDS[] = {
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME, 45, 45},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME, 46, 46},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__srv__ReadMetrics_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME, 45, 45},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME, 46, 46},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__srv__ReadMetrics_Event__TYPE_NAME, 43, 43},
      {cartographer_ros_msgs__srv__ReadMetrics_Event__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__srv__ReadMetrics_Event__REFERENCED_TYPE_DESCRIPTIONS, 9, 9},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH, cartographer_ros_msgs__msg__HistogramBucket__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__HistogramBucket__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__Metric__EXPECTED_HASH, cartographer_ros_msgs__msg__Metric__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__Metric__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricFamily__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricFamily__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = cartographer_ros_msgs__msg__MetricFamily__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricLabel__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = cartographer_ros_msgs__msg__MetricLabel__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__StatusResponse__EXPECTED_HASH, cartographer_ros_msgs__msg__StatusResponse__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = cartographer_ros_msgs__msg__StatusResponse__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[6].fields = cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[7].fields = cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[8].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
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
  "---\n"
  "cartographer_ros_msgs/StatusResponse status\n"
  "cartographer_ros_msgs/MetricFamily[] metric_families\n"
  "builtin_interfaces/Time timestamp";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__ReadMetrics__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__ReadMetrics__TYPE_NAME, 37, 37},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 725, 725},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__ReadMetrics_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__ReadMetrics_Request__TYPE_NAME, 45, 45},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__ReadMetrics_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__ReadMetrics_Response__TYPE_NAME, 46, 46},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__srv__ReadMetrics_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__srv__ReadMetrics_Event__TYPE_NAME, 43, 43},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__ReadMetrics__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[11];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 11, 11};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__ReadMetrics__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__Metric__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__msg__MetricFamily__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__msg__MetricLabel__get_individual_type_description_source(NULL);
    sources[6] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[7] = *cartographer_ros_msgs__srv__ReadMetrics_Event__get_individual_type_description_source(NULL);
    sources[8] = *cartographer_ros_msgs__srv__ReadMetrics_Request__get_individual_type_description_source(NULL);
    sources[9] = *cartographer_ros_msgs__srv__ReadMetrics_Response__get_individual_type_description_source(NULL);
    sources[10] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__ReadMetrics_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__ReadMetrics_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__ReadMetrics_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__ReadMetrics_Response__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__Metric__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__msg__MetricFamily__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__msg__MetricLabel__get_individual_type_description_source(NULL);
    sources[6] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__srv__ReadMetrics_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[10];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 10, 10};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__srv__ReadMetrics_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__Metric__get_individual_type_description_source(NULL);
    sources[4] = *cartographer_ros_msgs__msg__MetricFamily__get_individual_type_description_source(NULL);
    sources[5] = *cartographer_ros_msgs__msg__MetricLabel__get_individual_type_description_source(NULL);
    sources[6] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL);
    sources[7] = *cartographer_ros_msgs__srv__ReadMetrics_Request__get_individual_type_description_source(NULL);
    sources[8] = *cartographer_ros_msgs__srv__ReadMetrics_Response__get_individual_type_description_source(NULL);
    sources[9] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
