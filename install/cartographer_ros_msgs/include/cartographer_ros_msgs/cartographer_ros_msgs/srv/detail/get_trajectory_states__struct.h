// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from cartographer_ros_msgs:srv/GetTrajectoryStates.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cartographer_ros_msgs/srv/get_trajectory_states.h"


#ifndef CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__GET_TRAJECTORY_STATES__STRUCT_H_
#define CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__GET_TRAJECTORY_STATES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetTrajectoryStates in the package cartographer_ros_msgs.
typedef struct cartographer_ros_msgs__srv__GetTrajectoryStates_Request
{
  uint8_t structure_needs_at_least_one_member;
} cartographer_ros_msgs__srv__GetTrajectoryStates_Request;

// Struct for a sequence of cartographer_ros_msgs__srv__GetTrajectoryStates_Request.
typedef struct cartographer_ros_msgs__srv__GetTrajectoryStates_Request__Sequence
{
  cartographer_ros_msgs__srv__GetTrajectoryStates_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cartographer_ros_msgs__srv__GetTrajectoryStates_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "cartographer_ros_msgs/msg/detail/status_response__struct.h"
// Member 'trajectory_states'
#include "cartographer_ros_msgs/msg/detail/trajectory_states__struct.h"

/// Struct defined in srv/GetTrajectoryStates in the package cartographer_ros_msgs.
typedef struct cartographer_ros_msgs__srv__GetTrajectoryStates_Response
{
  cartographer_ros_msgs__msg__StatusResponse status;
  cartographer_ros_msgs__msg__TrajectoryStates trajectory_states;
} cartographer_ros_msgs__srv__GetTrajectoryStates_Response;

// Struct for a sequence of cartographer_ros_msgs__srv__GetTrajectoryStates_Response.
typedef struct cartographer_ros_msgs__srv__GetTrajectoryStates_Response__Sequence
{
  cartographer_ros_msgs__srv__GetTrajectoryStates_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cartographer_ros_msgs__srv__GetTrajectoryStates_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  cartographer_ros_msgs__srv__GetTrajectoryStates_Event__request__MAX_SIZE = 1
};
// response
enum
{
  cartographer_ros_msgs__srv__GetTrajectoryStates_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GetTrajectoryStates in the package cartographer_ros_msgs.
typedef struct cartographer_ros_msgs__srv__GetTrajectoryStates_Event
{
  service_msgs__msg__ServiceEventInfo info;
  cartographer_ros_msgs__srv__GetTrajectoryStates_Request__Sequence request;
  cartographer_ros_msgs__srv__GetTrajectoryStates_Response__Sequence response;
} cartographer_ros_msgs__srv__GetTrajectoryStates_Event;

// Struct for a sequence of cartographer_ros_msgs__srv__GetTrajectoryStates_Event.
typedef struct cartographer_ros_msgs__srv__GetTrajectoryStates_Event__Sequence
{
  cartographer_ros_msgs__srv__GetTrajectoryStates_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} cartographer_ros_msgs__srv__GetTrajectoryStates_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__GET_TRAJECTORY_STATES__STRUCT_H_
