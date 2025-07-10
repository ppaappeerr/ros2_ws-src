// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/Metric.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/metric__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__Metric__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa2, 0xe4, 0x53, 0x2a, 0xe6, 0x41, 0x45, 0x63,
      0x3f, 0x94, 0x4a, 0xc4, 0xde, 0x6b, 0x49, 0x65,
      0x66, 0xc2, 0x03, 0x22, 0xfe, 0xd7, 0xe4, 0x24,
      0x7a, 0x60, 0xbd, 0xca, 0x16, 0x95, 0x2a, 0x60,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "cartographer_ros_msgs/msg/detail/metric_label__functions.h"
#include "cartographer_ros_msgs/msg/detail/histogram_bucket__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH = {1, {
    0x3a, 0x4a, 0xb1, 0x33, 0x65, 0xf7, 0x0f, 0x3f,
    0x9d, 0xe5, 0x0a, 0x4c, 0x41, 0x39, 0x8a, 0xb5,
    0xe1, 0x0a, 0xa9, 0xf8, 0x34, 0xf8, 0x05, 0x03,
    0xc8, 0x56, 0x80, 0x35, 0xa3, 0x30, 0xb8, 0x6b,
  }};
static const rosidl_type_hash_t cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH = {1, {
    0x8b, 0xdd, 0x8b, 0x3d, 0x78, 0x1c, 0x0f, 0xcc,
    0xb4, 0x07, 0x84, 0x0d, 0x6e, 0x7a, 0x9c, 0x49,
    0x61, 0xf5, 0x1f, 0x3d, 0xcd, 0x15, 0x18, 0x16,
    0x52, 0x24, 0xbb, 0xa1, 0x70, 0xb1, 0xdb, 0x3f,
  }};
#endif

static char cartographer_ros_msgs__msg__Metric__TYPE_NAME[] = "cartographer_ros_msgs/msg/Metric";
static char cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME[] = "cartographer_ros_msgs/msg/HistogramBucket";
static char cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME[] = "cartographer_ros_msgs/msg/MetricLabel";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__Metric__FIELD_NAME__type[] = "type";
static char cartographer_ros_msgs__msg__Metric__FIELD_NAME__labels[] = "labels";
static char cartographer_ros_msgs__msg__Metric__FIELD_NAME__value[] = "value";
static char cartographer_ros_msgs__msg__Metric__FIELD_NAME__counts_by_bucket[] = "counts_by_bucket";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__Metric__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__Metric__FIELD_NAME__type, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__FIELD_NAME__labels, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__FIELD_NAME__value, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__FIELD_NAME__counts_by_bucket, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__msg__Metric__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__msg__Metric__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
      {cartographer_ros_msgs__msg__Metric__FIELDS, 4, 4},
    },
    {cartographer_ros_msgs__msg__Metric__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH, cartographer_ros_msgs__msg__HistogramBucket__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = cartographer_ros_msgs__msg__HistogramBucket__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricLabel__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__MetricLabel__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# 2018 The Cartographer Authors\n"
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
  "uint8 TYPE_COUNTER=0\n"
  "uint8 TYPE_GAUGE=1\n"
  "uint8 TYPE_HISTOGRAM=2\n"
  "\n"
  "uint8 type\n"
  "cartographer_ros_msgs/MetricLabel[] labels\n"
  "\n"
  "# TYPE_COUNTER or TYPE_GAUGE\n"
  "float64 value\n"
  "\n"
  "# TYPE_HISTOGRAM\n"
  "cartographer_ros_msgs/HistogramBucket[] counts_by_bucket";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__Metric__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 817, 817},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__Metric__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__Metric__get_individual_type_description_source(NULL),
    sources[1] = *cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__MetricLabel__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
