// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/nav_hint.hpp"


#ifndef CPP_PACKAGE__MSG__DETAIL__NAV_HINT__TRAITS_HPP_
#define CPP_PACKAGE__MSG__DETAIL__NAV_HINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "cpp_package/msg/detail/nav_hint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace cpp_package
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavHint & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavHint & msg,
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

  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavHint & msg, bool use_flow_style = false)
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
  const cpp_package::msg::NavHint & msg,
  std::ostream & out, size_t indentation = 0)
{
  cpp_package::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use cpp_package::msg::to_yaml() instead")]]
inline std::string to_yaml(const cpp_package::msg::NavHint & msg)
{
  return cpp_package::msg::to_yaml(msg);
}

template<>
inline const char * data_type<cpp_package::msg::NavHint>()
{
  return "cpp_package::msg::NavHint";
}

template<>
inline const char * name<cpp_package::msg::NavHint>()
{
  return "cpp_package/msg/NavHint";
}

template<>
struct has_fixed_size<cpp_package::msg::NavHint>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<cpp_package::msg::NavHint>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<cpp_package::msg::NavHint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CPP_PACKAGE__MSG__DETAIL__NAV_HINT__TRAITS_HPP_
