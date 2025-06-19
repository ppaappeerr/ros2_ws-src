// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/HistogramBucket.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/histogram_bucket__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__HistogramBucket__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3a, 0x4a, 0xb1, 0x33, 0x65, 0xf7, 0x0f, 0x3f,
      0x9d, 0xe5, 0x0a, 0x4c, 0x41, 0x39, 0x8a, 0xb5,
      0xe1, 0x0a, 0xa9, 0xf8, 0x34, 0xf8, 0x05, 0x03,
      0xc8, 0x56, 0x80, 0x35, 0xa3, 0x30, 0xb8, 0x6b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME[] = "cartographer_ros_msgs/msg/HistogramBucket";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__HistogramBucket__FIELD_NAME__bucket_boundary[] = "bucket_boundary";
static char cartographer_ros_msgs__msg__HistogramBucket__FIELD_NAME__count[] = "count";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__HistogramBucket__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__HistogramBucket__FIELD_NAME__bucket_boundary, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__HistogramBucket__FIELD_NAME__count, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__msg__HistogramBucket__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
      {cartographer_ros_msgs__msg__HistogramBucket__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
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
  "# A histogram bucket counts values x for which:\n"
  "#   previous_boundary < x <= bucket_boundary\n"
  "# holds.\n"
  "float64 bucket_boundary\n"
  "float64 count";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__HistogramBucket__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 720, 720},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__HistogramBucket__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__HistogramBucket__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
