// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/MetricFamily.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/metric_family__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__MetricFamily__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3b, 0x06, 0x55, 0x16, 0xbc, 0x1d, 0x35, 0x21,
      0x18, 0x30, 0x81, 0x1e, 0x9d, 0xce, 0x7f, 0xbd,
      0xe2, 0xf8, 0x54, 0xcd, 0xd4, 0x31, 0x2f, 0x9c,
      0x0a, 0xa4, 0x28, 0xb1, 0xf5, 0x71, 0x18, 0xc6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "cartographer_ros_msgs/msg/detail/metric_label__functions.h"
#include "cartographer_ros_msgs/msg/detail/histogram_bucket__functions.h"
#include "cartographer_ros_msgs/msg/detail/metric__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
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
static const rosidl_type_hash_t cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH = {1, {
    0x8b, 0xdd, 0x8b, 0x3d, 0x78, 0x1c, 0x0f, 0xcc,
    0xb4, 0x07, 0x84, 0x0d, 0x6e, 0x7a, 0x9c, 0x49,
    0x61, 0xf5, 0x1f, 0x3d, 0xcd, 0x15, 0x18, 0x16,
    0x52, 0x24, 0xbb, 0xa1, 0x70, 0xb1, 0xdb, 0x3f,
  }};
#endif

static char cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME[] = "cartographer_ros_msgs/msg/MetricFamily";
static char cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME[] = "cartographer_ros_msgs/msg/HistogramBucket";
static char cartographer_ros_msgs__msg__Metric__TYPE_NAME[] = "cartographer_ros_msgs/msg/Metric";
static char cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME[] = "cartographer_ros_msgs/msg/MetricLabel";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__MetricFamily__FIELD_NAME__name[] = "name";
static char cartographer_ros_msgs__msg__MetricFamily__FIELD_NAME__description[] = "description";
static char cartographer_ros_msgs__msg__MetricFamily__FIELD_NAME__metrics[] = "metrics";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__MetricFamily__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__MetricFamily__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricFamily__FIELD_NAME__description, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricFamily__FIELD_NAME__metrics, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__msg__MetricFamily__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__Metric__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__MetricLabel__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__msg__MetricFamily__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME, 38, 38},
      {cartographer_ros_msgs__msg__MetricFamily__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__msg__MetricFamily__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&cartographer_ros_msgs__msg__HistogramBucket__EXPECTED_HASH, cartographer_ros_msgs__msg__HistogramBucket__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = cartographer_ros_msgs__msg__HistogramBucket__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__Metric__EXPECTED_HASH, cartographer_ros_msgs__msg__Metric__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = cartographer_ros_msgs__msg__Metric__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&cartographer_ros_msgs__msg__MetricLabel__EXPECTED_HASH, cartographer_ros_msgs__msg__MetricLabel__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = cartographer_ros_msgs__msg__MetricLabel__get_type_description(NULL)->type_description.fields;
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
  "string name\n"
  "string description\n"
  "cartographer_ros_msgs/Metric[] metrics";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__MetricFamily__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__MetricFamily__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 650, 650},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__MetricFamily__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__MetricFamily__get_individual_type_description_source(NULL),
    sources[1] = *cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(NULL);
    sources[2] = *cartographer_ros_msgs__msg__Metric__get_individual_type_description_source(NULL);
    sources[3] = *cartographer_ros_msgs__msg__MetricLabel__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
