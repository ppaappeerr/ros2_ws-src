// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from cartographer_ros_msgs:msg/TrajectoryStates.idl
// generated code does not contain a copyright notice

#include "cartographer_ros_msgs/msg/detail/trajectory_states__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_cartographer_ros_msgs
const rosidl_type_hash_t *
cartographer_ros_msgs__msg__TrajectoryStates__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x11, 0x0c, 0x24, 0xec, 0xf1, 0x3f, 0x1f, 0xa1,
      0x44, 0xb1, 0x77, 0xa5, 0x5d, 0x56, 0xae, 0x67,
      0x69, 0x24, 0x02, 0xc4, 0x84, 0xae, 0x6b, 0x5f,
      0xe1, 0x99, 0x52, 0x53, 0xd8, 0xa7, 0x9c, 0x39,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME[] = "cartographer_ros_msgs/msg/TrajectoryStates";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char cartographer_ros_msgs__msg__TrajectoryStates__FIELD_NAME__header[] = "header";
static char cartographer_ros_msgs__msg__TrajectoryStates__FIELD_NAME__trajectory_id[] = "trajectory_id";
static char cartographer_ros_msgs__msg__TrajectoryStates__FIELD_NAME__trajectory_state[] = "trajectory_state";

static rosidl_runtime_c__type_description__Field cartographer_ros_msgs__msg__TrajectoryStates__FIELDS[] = {
  {
    {cartographer_ros_msgs__msg__TrajectoryStates__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__TrajectoryStates__FIELD_NAME__trajectory_id, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {cartographer_ros_msgs__msg__TrajectoryStates__FIELD_NAME__trajectory_state, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription cartographer_ros_msgs__msg__TrajectoryStates__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
cartographer_ros_msgs__msg__TrajectoryStates__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME, 42, 42},
      {cartographer_ros_msgs__msg__TrajectoryStates__FIELDS, 3, 3},
    },
    {cartographer_ros_msgs__msg__TrajectoryStates__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Copyright 2018 The Cartographer Authors\n"
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
  "uint8 ACTIVE = 0\n"
  "uint8 FINISHED = 1\n"
  "uint8 FROZEN = 2\n"
  "uint8 DELETED = 3\n"
  "\n"
  "std_msgs/Header header\n"
  "int32[] trajectory_id\n"
  "uint8[] trajectory_state";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
cartographer_ros_msgs__msg__TrajectoryStates__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {cartographer_ros_msgs__msg__TrajectoryStates__TYPE_NAME, 42, 42},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 732, 732},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
cartographer_ros_msgs__msg__TrajectoryStates__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *cartographer_ros_msgs__msg__TrajectoryStates__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
