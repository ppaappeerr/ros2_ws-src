// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/obstacle_feedback.hpp"


#ifndef CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__TRAITS_HPP_
#define CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cpp_package/msg/detail/obstacle_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace cpp_package
{

namespace msg
{

inline void to_flow_style_yaml(
  const ObstacleFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: min_left
  {
    out << "min_left: ";
    rosidl_generator_traits::value_to_yaml(msg.min_left, out);
    out << ", ";
  }

  // member: min_center
  {
    out << "min_center: ";
    rosidl_generator_traits::value_to_yaml(msg.min_center, out);
    out << ", ";
  }

  // member: min_right
  {
    out << "min_right: ";
    rosidl_generator_traits::value_to_yaml(msg.min_right, out);
    out << ", ";
  }

  // member: level_left
  {
    out << "level_left: ";
    rosidl_generator_traits::value_to_yaml(msg.level_left, out);
    out << ", ";
  }

  // member: level_center
  {
    out << "level_center: ";
    rosidl_generator_traits::value_to_yaml(msg.level_center, out);
    out << ", ";
  }

  // member: level_right
  {
    out << "level_right: ";
    rosidl_generator_traits::value_to_yaml(msg.level_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ObstacleFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: min_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_left: ";
    rosidl_generator_traits::value_to_yaml(msg.min_left, out);
    out << "\n";
  }

  // member: min_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_center: ";
    rosidl_generator_traits::value_to_yaml(msg.min_center, out);
    out << "\n";
  }

  // member: min_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_right: ";
    rosidl_generator_traits::value_to_yaml(msg.min_right, out);
    out << "\n";
  }

  // member: level_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level_left: ";
    rosidl_generator_traits::value_to_yaml(msg.level_left, out);
    out << "\n";
  }

  // member: level_center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level_center: ";
    rosidl_generator_traits::value_to_yaml(msg.level_center, out);
    out << "\n";
  }

  // member: level_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "level_right: ";
    rosidl_generator_traits::value_to_yaml(msg.level_right, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ObstacleFeedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace cpp_package

namespace rosidl_generator_traits
{

[[deprecated("use cpp_package::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const cpp_package::msg::ObstacleFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  cpp_package::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cpp_package::msg::to_yaml() instead")]]
inline std::string to_yaml(const cpp_package::msg::ObstacleFeedback & msg)
{
  return cpp_package::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cpp_package::msg::ObstacleFeedback>()
{
  return "cpp_package::msg::ObstacleFeedback";
}

template<>
inline const char * name<cpp_package::msg::ObstacleFeedback>()
{
  return "cpp_package/msg/ObstacleFeedback";
}

template<>
struct has_fixed_size<cpp_package::msg::ObstacleFeedback>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<cpp_package::msg::ObstacleFeedback>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<cpp_package::msg::ObstacleFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__TRAITS_HPP_
