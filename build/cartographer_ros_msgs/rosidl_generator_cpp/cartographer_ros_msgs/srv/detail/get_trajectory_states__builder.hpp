// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cartographer_ros_msgs:srv/GetTrajectoryStates.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"


#ifndef CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__GET_TRAJECTORY_STATES__BUILDER_HPP_
#define CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__GET_TRAJECTORY_STATES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cartographer_ros_msgs/srv/detail/get_trajectory_states__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cartographer_ros_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::GetTrajectoryStates_Request>()
{
  return ::cartographer_ros_msgs::srv::GetTrajectoryStates_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace cartographer_ros_msgs


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTrajectoryStates_Response_trajectory_states
{
public:
  explicit Init_GetTrajectoryStates_Response_trajectory_states(::cartographer_ros_msgs::srv::GetTrajectoryStates_Response & msg)
  : msg_(msg)
  {}
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Response trajectory_states(::cartographer_ros_msgs::srv::GetTrajectoryStates_Response::_trajectory_states_type arg)
  {
    msg_.trajectory_states = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Response msg_;
};

class Init_GetTrajectoryStates_Response_status
{
public:
  Init_GetTrajectoryStates_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTrajectoryStates_Response_trajectory_states status(::cartographer_ros_msgs::srv::GetTrajectoryStates_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GetTrajectoryStates_Response_trajectory_states(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::GetTrajectoryStates_Response>()
{
  return cartographer_ros_msgs::srv::builder::Init_GetTrajectoryStates_Response_status();
}

}  // namespace cartographer_ros_msgs


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTrajectoryStates_Event_response
{
public:
  explicit Init_GetTrajectoryStates_Event_response(::cartographer_ros_msgs::srv::GetTrajectoryStates_Event & msg)
  : msg_(msg)
  {}
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Event response(::cartographer_ros_msgs::srv::GetTrajectoryStates_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Event msg_;
};

class Init_GetTrajectoryStates_Event_request
{
public:
  explicit Init_GetTrajectoryStates_Event_request(::cartographer_ros_msgs::srv::GetTrajectoryStates_Event & msg)
  : msg_(msg)
  {}
  Init_GetTrajectoryStates_Event_response request(::cartographer_ros_msgs::srv::GetTrajectoryStates_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetTrajectoryStates_Event_response(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Event msg_;
};

class Init_GetTrajectoryStates_Event_info
{
public:
  Init_GetTrajectoryStates_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTrajectoryStates_Event_request info(::cartographer_ros_msgs::srv::GetTrajectoryStates_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetTrajectoryStates_Event_request(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::GetTrajectoryStates_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::GetTrajectoryStates_Event>()
{
  return cartographer_ros_msgs::srv::builder::Init_GetTrajectoryStates_Event_info();
}

}  // namespace cartographer_ros_msgs

#endif  // CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__GET_TRAJECTORY_STATES__BUILDER_HPP_
