// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cartographer_ros_msgs:srv/TrajectoryQuery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cartographer_ros_msgs/srv/trajectory_query.hpp"


#ifndef CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__TRAJECTORY_QUERY__BUILDER_HPP_
#define CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__TRAJECTORY_QUERY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cartographer_ros_msgs/srv/detail/trajectory_query__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_TrajectoryQuery_Request_trajectory_id
{
public:
  Init_TrajectoryQuery_Request_trajectory_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Request trajectory_id(::cartographer_ros_msgs::srv::TrajectoryQuery_Request::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::TrajectoryQuery_Request>()
{
  return cartographer_ros_msgs::srv::builder::Init_TrajectoryQuery_Request_trajectory_id();
}

}  // namespace cartographer_ros_msgs


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_TrajectoryQuery_Response_trajectory
{
public:
  explicit Init_TrajectoryQuery_Response_trajectory(::cartographer_ros_msgs::srv::TrajectoryQuery_Response & msg)
  : msg_(msg)
  {}
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Response trajectory(::cartographer_ros_msgs::srv::TrajectoryQuery_Response::_trajectory_type arg)
  {
    msg_.trajectory = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Response msg_;
};

class Init_TrajectoryQuery_Response_status
{
public:
  Init_TrajectoryQuery_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryQuery_Response_trajectory status(::cartographer_ros_msgs::srv::TrajectoryQuery_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TrajectoryQuery_Response_trajectory(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::TrajectoryQuery_Response>()
{
  return cartographer_ros_msgs::srv::builder::Init_TrajectoryQuery_Response_status();
}

}  // namespace cartographer_ros_msgs


namespace cartographer_ros_msgs
{

namespace srv
{

namespace builder
{

class Init_TrajectoryQuery_Event_response
{
public:
  explicit Init_TrajectoryQuery_Event_response(::cartographer_ros_msgs::srv::TrajectoryQuery_Event & msg)
  : msg_(msg)
  {}
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Event response(::cartographer_ros_msgs::srv::TrajectoryQuery_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Event msg_;
};

class Init_TrajectoryQuery_Event_request
{
public:
  explicit Init_TrajectoryQuery_Event_request(::cartographer_ros_msgs::srv::TrajectoryQuery_Event & msg)
  : msg_(msg)
  {}
  Init_TrajectoryQuery_Event_response request(::cartographer_ros_msgs::srv::TrajectoryQuery_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_TrajectoryQuery_Event_response(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Event msg_;
};

class Init_TrajectoryQuery_Event_info
{
public:
  Init_TrajectoryQuery_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryQuery_Event_request info(::cartographer_ros_msgs::srv::TrajectoryQuery_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_TrajectoryQuery_Event_request(msg_);
  }

private:
  ::cartographer_ros_msgs::srv::TrajectoryQuery_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::cartographer_ros_msgs::srv::TrajectoryQuery_Event>()
{
  return cartographer_ros_msgs::srv::builder::Init_TrajectoryQuery_Event_info();
}

}  // namespace cartographer_ros_msgs

#endif  // CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__TRAJECTORY_QUERY__BUILDER_HPP_
