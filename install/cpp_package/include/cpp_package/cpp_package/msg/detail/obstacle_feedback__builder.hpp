// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/obstacle_feedback.hpp"


#ifndef CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__BUILDER_HPP_
#define CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cpp_package/msg/detail/obstacle_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cpp_package
{

namespace msg
{

namespace builder
{

class Init_ObstacleFeedback_level_right
{
public:
  explicit Init_ObstacleFeedback_level_right(::cpp_package::msg::ObstacleFeedback & msg)
  : msg_(msg)
  {}
  ::cpp_package::msg::ObstacleFeedback level_right(::cpp_package::msg::ObstacleFeedback::_level_right_type arg)
  {
    msg_.level_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

class Init_ObstacleFeedback_level_center
{
public:
  explicit Init_ObstacleFeedback_level_center(::cpp_package::msg::ObstacleFeedback & msg)
  : msg_(msg)
  {}
  Init_ObstacleFeedback_level_right level_center(::cpp_package::msg::ObstacleFeedback::_level_center_type arg)
  {
    msg_.level_center = std::move(arg);
    return Init_ObstacleFeedback_level_right(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

class Init_ObstacleFeedback_level_left
{
public:
  explicit Init_ObstacleFeedback_level_left(::cpp_package::msg::ObstacleFeedback & msg)
  : msg_(msg)
  {}
  Init_ObstacleFeedback_level_center level_left(::cpp_package::msg::ObstacleFeedback::_level_left_type arg)
  {
    msg_.level_left = std::move(arg);
    return Init_ObstacleFeedback_level_center(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

class Init_ObstacleFeedback_min_right
{
public:
  explicit Init_ObstacleFeedback_min_right(::cpp_package::msg::ObstacleFeedback & msg)
  : msg_(msg)
  {}
  Init_ObstacleFeedback_level_left min_right(::cpp_package::msg::ObstacleFeedback::_min_right_type arg)
  {
    msg_.min_right = std::move(arg);
    return Init_ObstacleFeedback_level_left(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

class Init_ObstacleFeedback_min_center
{
public:
  explicit Init_ObstacleFeedback_min_center(::cpp_package::msg::ObstacleFeedback & msg)
  : msg_(msg)
  {}
  Init_ObstacleFeedback_min_right min_center(::cpp_package::msg::ObstacleFeedback::_min_center_type arg)
  {
    msg_.min_center = std::move(arg);
    return Init_ObstacleFeedback_min_right(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

class Init_ObstacleFeedback_min_left
{
public:
  explicit Init_ObstacleFeedback_min_left(::cpp_package::msg::ObstacleFeedback & msg)
  : msg_(msg)
  {}
  Init_ObstacleFeedback_min_center min_left(::cpp_package::msg::ObstacleFeedback::_min_left_type arg)
  {
    msg_.min_left = std::move(arg);
    return Init_ObstacleFeedback_min_center(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

class Init_ObstacleFeedback_header
{
public:
  Init_ObstacleFeedback_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleFeedback_min_left header(::cpp_package::msg::ObstacleFeedback::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ObstacleFeedback_min_left(msg_);
  }

private:
  ::cpp_package::msg::ObstacleFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_package::msg::ObstacleFeedback>()
{
  return cpp_package::msg::builder::Init_ObstacleFeedback_header();
}

}  // namespace cpp_package

#endif  // CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__BUILDER_HPP_
