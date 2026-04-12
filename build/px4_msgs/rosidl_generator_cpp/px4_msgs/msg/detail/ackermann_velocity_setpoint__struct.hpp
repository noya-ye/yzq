// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/AckermannVelocitySetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__AckermannVelocitySetpoint __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__AckermannVelocitySetpoint __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AckermannVelocitySetpoint_
{
  using Type = AckermannVelocitySetpoint_<ContainerAllocator>;

  explicit AckermannVelocitySetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      std::fill<typename std::array<float, 2>::iterator, float>(this->velocity_ned.begin(), this->velocity_ned.end(), 0.0f);
      this->backwards = false;
    }
  }

  explicit AckermannVelocitySetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : velocity_ned(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      std::fill<typename std::array<float, 2>::iterator, float>(this->velocity_ned.begin(), this->velocity_ned.end(), 0.0f);
      this->backwards = false;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _velocity_ned_type =
    std::array<float, 2>;
  _velocity_ned_type velocity_ned;
  using _backwards_type =
    bool;
  _backwards_type backwards;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__velocity_ned(
    const std::array<float, 2> & _arg)
  {
    this->velocity_ned = _arg;
    return *this;
  }
  Type & set__backwards(
    const bool & _arg)
  {
    this->backwards = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__AckermannVelocitySetpoint
    std::shared_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__AckermannVelocitySetpoint
    std::shared_ptr<px4_msgs::msg::AckermannVelocitySetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AckermannVelocitySetpoint_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->velocity_ned != other.velocity_ned) {
      return false;
    }
    if (this->backwards != other.backwards) {
      return false;
    }
    return true;
  }
  bool operator!=(const AckermannVelocitySetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AckermannVelocitySetpoint_

// alias to use template instance with default allocator
using AckermannVelocitySetpoint =
  px4_msgs::msg::AckermannVelocitySetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__STRUCT_HPP_
