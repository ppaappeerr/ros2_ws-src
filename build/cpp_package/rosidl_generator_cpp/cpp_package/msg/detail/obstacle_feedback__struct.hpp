// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cpp_package/msg/obstacle_feedback.hpp"


#ifndef CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__STRUCT_HPP_
#define CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__STRUCT_HPP_

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
# define DEPRECATED__cpp_package__msg__ObstacleFeedback __attribute__((deprecated))
#else
# define DEPRECATED__cpp_package__msg__ObstacleFeedback __declspec(deprecated)
#endif

namespace cpp_package
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleFeedback_
{
  using Type = ObstacleFeedback_<ContainerAllocator>;

  explicit ObstacleFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->min_left = 0.0f;
      this->min_center = 0.0f;
      this->min_right = 0.0f;
      this->level_left = 0;
      this->level_center = 0;
      this->level_right = 0;
    }
  }

  explicit ObstacleFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->min_left = 0.0f;
      this->min_center = 0.0f;
      this->min_right = 0.0f;
      this->level_left = 0;
      this->level_center = 0;
      this->level_right = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _min_left_type =
    float;
  _min_left_type min_left;
  using _min_center_type =
    float;
  _min_center_type min_center;
  using _min_right_type =
    float;
  _min_right_type min_right;
  using _level_left_type =
    uint8_t;
  _level_left_type level_left;
  using _level_center_type =
    uint8_t;
  _level_center_type level_center;
  using _level_right_type =
    uint8_t;
  _level_right_type level_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__min_left(
    const float & _arg)
  {
    this->min_left = _arg;
    return *this;
  }
  Type & set__min_center(
    const float & _arg)
  {
    this->min_center = _arg;
    return *this;
  }
  Type & set__min_right(
    const float & _arg)
  {
    this->min_right = _arg;
    return *this;
  }
  Type & set__level_left(
    const uint8_t & _arg)
  {
    this->level_left = _arg;
    return *this;
  }
  Type & set__level_center(
    const uint8_t & _arg)
  {
    this->level_center = _arg;
    return *this;
  }
  Type & set__level_right(
    const uint8_t & _arg)
  {
    this->level_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cpp_package::msg::ObstacleFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const cpp_package::msg::ObstacleFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cpp_package::msg::ObstacleFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cpp_package::msg::ObstacleFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cpp_package__msg__ObstacleFeedback
    std::shared_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cpp_package__msg__ObstacleFeedback
    std::shared_ptr<cpp_package::msg::ObstacleFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleFeedback_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->min_left != other.min_left) {
      return false;
    }
    if (this->min_center != other.min_center) {
      return false;
    }
    if (this->min_right != other.min_right) {
      return false;
    }
    if (this->level_left != other.level_left) {
      return false;
    }
    if (this->level_center != other.level_center) {
      return false;
    }
    if (this->level_right != other.level_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleFeedback_

// alias to use template instance with default allocator
using ObstacleFeedback =
  cpp_package::msg::ObstacleFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace cpp_package

#endif  // CPP_PACKAGE__MSG__DETAIL__OBSTACLE_FEEDBACK__STRUCT_HPP_
