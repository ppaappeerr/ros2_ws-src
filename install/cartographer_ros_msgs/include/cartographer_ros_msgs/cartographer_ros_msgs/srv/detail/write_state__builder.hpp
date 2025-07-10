// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cartographer_ros_msgs:srv/WriteState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cartographer_ros_msgs/srv/write_state.hpp"


#ifndef CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__WRITE_STATE__BUILDER_HPP_
#define CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__WRITE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cartographer_ros_msgs/srv/detail/write_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_WriteState_Request_include_unfinished_submaps
{
public:
  explicit Init_WriteState_Request_include_unfinished_submaps(::cartographer_ros_msgs::srv::WriteState_Request & msg)
  : msg_(msg)
  {}
  ::cartographer_ros_msgs::srv::WriteState_Request include_unfinished_submaps(::cartographer_ros_msgs::srv::WriteState_Request::_include_unfinished_submaps_type arg)
  {
    msg_.include_unfinished_submaps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::WriteState_Request msg_;
};

class Init_WriteState_Request_filename
{
public:
  Init_WriteState_Request_filename()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WriteState_Request_include_unfinished_submaps filename(::cartographer_ros_msgs::srv::WriteState_Request::_filename_type arg)
  {
    msg_.filename = std::move(arg);
    return Init_WriteState_Request_include_unfinished_submaps(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::WriteState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::WriteState_Request>()
{
  return cartographer_ros_msgs::srv::builder::Init_WriteState_Request_filename();
}

}  // namespace cartographer_ros_msgs


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_WriteState_Response_status
{
public:
  Init_WriteState_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cartographer_ros_msgs::srv::WriteState_Response status(::cartographer_ros_msgs::srv::WriteState_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::WriteState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::WriteState_Response>()
{
  return cartographer_ros_msgs::srv::builder::Init_WriteState_Response_status();
}

}  // namespace cartographer_ros_msgs


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_WriteState_Event_response
{
public:
  explicit Init_WriteState_Event_response(::cartographer_ros_msgs::srv::WriteState_Event & msg)
  : msg_(msg)
  {}
  ::cartographer_ros_msgs::srv::WriteState_Event response(::cartographer_ros_msgs::srv::WriteState_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::WriteState_Event msg_;
};

class Init_WriteState_Event_request
{
public:
  explicit Init_WriteState_Event_request(::cartographer_ros_msgs::srv::WriteState_Event & msg)
  : msg_(msg)
  {}
  Init_WriteState_Event_response request(::cartographer_ros_msgs::srv::WriteState_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_WriteState_Event_response(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::WriteState_Event msg_;
};

class Init_WriteState_Event_info
{
public:
  Init_WriteState_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WriteState_Event_request info(::cartographer_ros_msgs::srv::WriteState_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_WriteState_Event_request(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::WriteState_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::WriteState_Event>()
{
  return cartographer_ros_msgs::srv::builder::Init_WriteState_Event_info();
}

}  // namespace cartographer_ros_msgs

#endif  // CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__WRITE_STATE__BUILDER_HPP_
