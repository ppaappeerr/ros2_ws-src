// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/nav_hint.hpp"


#ifndef CPP_PACKAGE__MSG__DETAIL__NAV_HINT__STRUCT_HPP_
#define CPP_PACKAGE__MSG__DETAIL__NAV_HINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cpp_package__msg__NavHint __attribute__((deprecated))
#else
# define DEPRECATED__cpp_package__msg__NavHint __declspec(deprecated)
#endif

namespace cpp_package
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavHint_
{
  using Type = NavHint_<ContainerAllocator>;

  explicit NavHint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action = "";
    }
  }

  explicit NavHint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    action(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->action = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _action_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _action_type action;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__action(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->action = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cpp_package::msg::NavHint_<ContainerAllocator> *;
  using ConstRawPtr =
    const cpp_package::msg::NavHint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cpp_package::msg::NavHint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cpp_package::msg::NavHint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cpp_package::msg::NavHint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cpp_package::msg::NavHint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cpp_package::msg::NavHint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cpp_package::msg::NavHint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cpp_package::msg::NavHint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cpp_package::msg::NavHint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cpp_package__msg__NavHint
    std::shared_ptr<cpp_package::msg::NavHint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cpp_package__msg__NavHint
    std::shared_ptr<cpp_package::msg::NavHint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavHint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavHint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavHint_

// alias to use template instance with default allocator
using NavHint =
  cpp_package::msg::NavHint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cpp_package

#endif  // CPP_PACKAGE__MSG__DETAIL__NAV_HINT__STRUCT_HPP_
