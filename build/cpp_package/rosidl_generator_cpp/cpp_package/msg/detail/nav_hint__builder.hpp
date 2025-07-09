// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/nav_hint.hpp"


#ifndef CPP_PACKAGE__MSG__DETAIL__NAV_HINT__BUILDER_HPP_
#define CPP_PACKAGE__MSG__DETAIL__NAV_HINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "cpp_package/msg/detail/nav_hint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace cpp_package
{

namespace msg
{

namespace builder
{

class Init_NavHint_action
{
public:
  explicit Init_NavHint_action(::cpp_package::msg::NavHint & msg)
  : msg_(msg)
  {}
  ::cpp_package::msg::NavHint action(::cpp_package::msg::NavHint::_action_type arg)
  {
    msg_.action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::cpp_package::msg::NavHint msg_;
};

class Init_NavHint_header
{
public:
  Init_NavHint_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavHint_action header(::cpp_package::msg::NavHint::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavHint_action(msg_);
  }

private:
  ::cpp_package::msg::NavHint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::cpp_package::msg::NavHint>()
{
  return cpp_package::msg::builder::Init_NavHint_header();
}

}  // namespace cpp_package

#endif  // CPP_PACKAGE__MSG__DETAIL__NAV_HINT__BUILDER_HPP_
