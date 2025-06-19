// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/StatusResponse.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/status_response__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__StatusResponse__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x95, 0xaa, 0x98, 0xef, 0xf3, 0x25, 0xb4, 0x01,
      0xaf, 0x49, 0x25, 0x03, 0x96, 0xad, 0x39, 0x2d,
      0x30, 0x6c, 0xd5, 0x00, 0x18, 0x81, 0x85, 0x7a,
      0x01, 0x6e, 0x22, 0x94, 0x68, 0x77, 0x8f, 0x36,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME[] = "cartographer_ros_msgs/msg/StatusResponse";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__StatusResponse__FIELD_NAME__code[] = "code";
static char cartographer_ros_msgs__msg__StatusResponse__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__StatusResponse__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__StatusResponse__FIELD_NAME__code, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__StatusResponse__FIELD_NAME__message, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__msg__StatusResponse__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
      {cartographer_ros_msgs__msg__StatusResponse__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
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
  "# A common message type to indicate the outcome of a service call.\n"
  "uint8 code\n"
  "string message";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__StatusResponse__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 683, 683},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__StatusResponse__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__StatusResponse__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
