// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/DifferentialVelocitySetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_VELOCITY_SETPOINT__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_VELOCITY_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__DifferentialVelocitySetpoint __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__DifferentialVelocitySetpoint __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DifferentialVelocitySetpoint_
{
  using Type = DifferentialVelocitySetpoint_<ContainerAllocator>;

  explicit DifferentialVelocitySetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->speed = 0.0f;
      this->bearing = 0.0f;
    }
  }

  explicit DifferentialVelocitySetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->speed = 0.0f;
      this->bearing = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _speed_type =
    float;
  _speed_type speed;
  using _bearing_type =
    float;
  _bearing_type bearing;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__bearing(
    const float & _arg)
  {
    this->bearing = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__DifferentialVelocitySetpoint
    std::shared_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__DifferentialVelocitySetpoint
    std::shared_ptr<px4_msgs::msg::DifferentialVelocitySetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DifferentialVelocitySetpoint_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->bearing != other.bearing) {
      return false;
    }
    return true;
  }
  bool operator!=(const DifferentialVelocitySetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DifferentialVelocitySetpoint_

// alias to use template instance with default allocator
using DifferentialVelocitySetpoint =
  px4_msgs::msg::DifferentialVelocitySetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_VELOCITY_SETPOINT__STRUCT_HPP_
