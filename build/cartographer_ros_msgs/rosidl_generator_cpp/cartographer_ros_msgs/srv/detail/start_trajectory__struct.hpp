// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from cartographer_ros_msgs:srv/StartTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "cartographer_ros_msgs/srv/start_trajectory.hpp"


#ifndef CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__START_TRAJECTORY__STRUCT_HPP_
#define CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__START_TRAJECTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'initial_pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Request __attribute__((deprecated))
#else
# define DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Request __declspec(deprecated)
#endif

namespace cartographer_ros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartTrajectory_Request_
{
  using Type = StartTrajectory_Request_<ContainerAllocator>;

  explicit StartTrajectory_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : initial_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->configuration_directory = "";
      this->configuration_basename = "";
      this->use_initial_pose = false;
      this->relative_to_trajectory_id = 0l;
    }
  }

  explicit StartTrajectory_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : configuration_directory(_alloc),
    configuration_basename(_alloc),
    initial_pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->configuration_directory = "";
      this->configuration_basename = "";
      this->use_initial_pose = false;
      this->relative_to_trajectory_id = 0l;
    }
  }

  // field types and members
  using _configuration_directory_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _configuration_directory_type configuration_directory;
  using _configuration_basename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _configuration_basename_type configuration_basename;
  using _use_initial_pose_type =
    bool;
  _use_initial_pose_type use_initial_pose;
  using _initial_pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _initial_pose_type initial_pose;
  using _relative_to_trajectory_id_type =
    int32_t;
  _relative_to_trajectory_id_type relative_to_trajectory_id;

  // setters for named parameter idiom
  Type & set__configuration_directory(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->configuration_directory = _arg;
    return *this;
  }
  Type & set__configuration_basename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->configuration_basename = _arg;
    return *this;
  }
  Type & set__use_initial_pose(
    const bool & _arg)
  {
    this->use_initial_pose = _arg;
    return *this;
  }
  Type & set__initial_pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->initial_pose = _arg;
    return *this;
  }
  Type & set__relative_to_trajectory_id(
    const int32_t & _arg)
  {
    this->relative_to_trajectory_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Request
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Request
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartTrajectory_Request_ & other) const
  {
    if (this->configuration_directory != other.configuration_directory) {
      return false;
    }
    if (this->configuration_basename != other.configuration_basename) {
      return false;
    }
    if (this->use_initial_pose != other.use_initial_pose) {
      return false;
    }
    if (this->initial_pose != other.initial_pose) {
      return false;
    }
    if (this->relative_to_trajectory_id != other.relative_to_trajectory_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartTrajectory_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartTrajectory_Request_

// alias to use template instance with default allocator
using StartTrajectory_Request =
  cartographer_ros_msgs::srv::StartTrajectory_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cartographer_ros_msgs


// Include directives for member types
// Member 'status'
#include "cartographer_ros_msgs/msg/detail/status_response__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Response __attribute__((deprecated))
#else
# define DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Response __declspec(deprecated)
#endif

namespace cartographer_ros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartTrajectory_Response_
{
  using Type = StartTrajectory_Response_<ContainerAllocator>;

  explicit StartTrajectory_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0l;
    }
  }

  explicit StartTrajectory_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0l;
    }
  }

  // field types and members
  using _status_type =
    cartographer_ros_msgs::msg::StatusResponse_<ContainerAllocator>;
  _status_type status;
  using _trajectory_id_type =
    int32_t;
  _trajectory_id_type trajectory_id;

  // setters for named parameter idiom
  Type & set__status(
    const cartographer_ros_msgs::msg::StatusResponse_<ContainerAllocator> & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__trajectory_id(
    const int32_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Response
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Response
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartTrajectory_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartTrajectory_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartTrajectory_Response_

// alias to use template instance with default allocator
using StartTrajectory_Response =
  cartographer_ros_msgs::srv::StartTrajectory_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cartographer_ros_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Event __attribute__((deprecated))
#else
# define DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Event __declspec(deprecated)
#endif

namespace cartographer_ros_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct StartTrajectory_Event_
{
  using Type = StartTrajectory_Event_<ContainerAllocator>;

  explicit StartTrajectory_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit StartTrajectory_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cartographer_ros_msgs::srv::StartTrajectory_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<cartographer_ros_msgs::srv::StartTrajectory_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Event
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__cartographer_ros_msgs__srv__StartTrajectory_Event
    std::shared_ptr<cartographer_ros_msgs::srv::StartTrajectory_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const StartTrajectory_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const StartTrajectory_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct StartTrajectory_Event_

// alias to use template instance with default allocator
using StartTrajectory_Event =
  cartographer_ros_msgs::srv::StartTrajectory_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace cartographer_ros_msgs

namespace cartographer_ros_msgs
{

namespace srv
{

struct StartTrajectory
{
  using Request = cartographer_ros_msgs::srv::StartTrajectory_Request;
  using Response = cartographer_ros_msgs::srv::StartTrajectory_Response;
  using Event = cartographer_ros_msgs::srv::StartTrajectory_Event;
};

}  // namespace srv

}  // namespace cartographer_ros_msgs

#endif  // CARTOGRAPHER_ROS_MSGS__SRV__DETAIL__START_TRAJECTORY__STRUCT_HPP_
