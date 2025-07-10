// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/BagfileProgress.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/bagfile_progress__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__BagfileProgress__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x69, 0x4c, 0x80, 0x88, 0xec, 0xbf, 0x68, 0x36,
      0xb6, 0x4e, 0xda, 0x85, 0x94, 0x15, 0x3d, 0xef,
      0x50, 0x78, 0x31, 0xa7, 0xb1, 0xd3, 0x14, 0x44,
      0xe1, 0xa3, 0x8e, 0x86, 0x5b, 0xd9, 0x57, 0xe6,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cartographer_ros_msgs__msg__BagfileProgress__TYPE_NAME[] = "cartographer_ros_msgs/msg/BagfileProgress";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__current_bagfile_name[] = "current_bagfile_name";
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__current_bagfile_id[] = "current_bagfile_id";
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__total_bagfiles[] = "total_bagfiles";
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__total_messages[] = "total_messages";
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__processed_messages[] = "processed_messages";
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__total_seconds[] = "total_seconds";
static char cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__processed_seconds[] = "processed_seconds";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__BagfileProgress__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__current_bagfile_name, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__current_bagfile_id, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__total_bagfiles, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__total_messages, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__processed_messages, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__total_seconds, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__BagfileProgress__FIELD_NAME__processed_seconds, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__msg__BagfileProgress__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__BagfileProgress__TYPE_NAME, 41, 41},
      {cartographer_ros_msgs__msg__BagfileProgress__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "#\n"
  "# Licensed under the Apache License, Version 2.0 (the 'License');\n"
  "# you may not use this file except in compliance with the License.\n"
  "# You may obtain a copy of the License at\n"
  "#\n"
  "#      http://www.apache.org/licenses/LICENSE-2.0\n"
  "#\n"
  "# Unless required by applicable law or agreed to in writing, software\n"
  "# distributed under the License is distributed on an 'AS IS' BASIS,\n"
  "# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
  "# See the License for the specific language governing permissions and\n"
  "# limitations under the License.\n"
  "\n"
  "\n"
  "# Contains general information about the bagfiles processing progress\n"
  "\n"
  "string current_bagfile_name\n"
  "uint32 current_bagfile_id\n"
  "uint32 total_bagfiles\n"
  "uint32 total_messages\n"
  "uint32 processed_messages\n"
  "float32 total_seconds\n"
  "float32 processed_seconds";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__BagfileProgress__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__BagfileProgress__TYPE_NAME, 41, 41},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 792, 792},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__BagfileProgress__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__BagfileProgress__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
