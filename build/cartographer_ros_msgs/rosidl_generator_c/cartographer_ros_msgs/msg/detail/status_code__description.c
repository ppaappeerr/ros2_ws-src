// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/StatusCode.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/status_code__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__StatusCode__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc9, 0x40, 0x95, 0x39, 0xf9, 0x2c, 0x15, 0x7f,
      0x90, 0x23, 0xed, 0x72, 0xff, 0x50, 0xf8, 0xcb,
      0xef, 0xd2, 0x2a, 0x89, 0x1f, 0xb2, 0x40, 0x8b,
      0x51, 0x41, 0xcc, 0xc3, 0x5e, 0x35, 0xfa, 0xad,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char cartographer_ros_msgs__msg__StatusCode__TYPE_NAME[] = "cartographer_ros_msgs/msg/StatusCode";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__StatusCode__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__StatusCode__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__StatusCode__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
cartographer_ros_msgs__msg__StatusCode__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__StatusCode__TYPE_NAME, 36, 36},
      {cartographer_ros_msgs__msg__StatusCode__FIELDS, 1, 1},
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
  "# Definition of status code constants equivalent to the gRPC API.\n"
  "# https://developers.google.com/maps-booking/reference/grpc-api/status_codes\n"
  "uint8 OK=0\n"
  "uint8 CANCELLED=1\n"
  "uint8 UNKNOWN=2\n"
  "uint8 INVALID_ARGUMENT=3\n"
  "uint8 DEADLINE_EXCEEDED=4\n"
  "uint8 NOT_FOUND=5\n"
  "uint8 ALREADY_EXISTS=6\n"
  "uint8 PERMISSION_DENIED=7\n"
  "uint8 RESOURCE_EXHAUSTED=8\n"
  "uint8 FAILED_PRECONDITION=9\n"
  "uint8 ABORTED=10\n"
  "uint8 OUT_OF_RANGE=11\n"
  "uint8 UNIMPLEMENTED=12\n"
  "uint8 INTERNAL=13\n"
  "uint8 UNAVAILABLE=14\n"
  "uint8 DATA_LOSS=15";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__StatusCode__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__StatusCode__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1071, 1071},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__StatusCode__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__StatusCode__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
