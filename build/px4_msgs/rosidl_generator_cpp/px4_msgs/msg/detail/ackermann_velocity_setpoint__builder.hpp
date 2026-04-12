// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/AckermannVelocitySetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/ackermann_velocity_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_AckermannVelocitySetpoint_backwards
{
public:
  explicit Init_AckermannVelocitySetpoint_backwards(::px4_msgs::msg::AckermannVelocitySetpoint & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::AckermannVelocitySetpoint backwards(::px4_msgs::msg::AckermannVelocitySetpoint::_backwards_type arg)
  {
    msg_.backwards = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::AckermannVelocitySetpoint msg_;
};

class Init_AckermannVelocitySetpoint_velocity_ned
{
public:
  explicit Init_AckermannVelocitySetpoint_velocity_ned(::px4_msgs::msg::AckermannVelocitySetpoint & msg)
  : msg_(msg)
  {}
  Init_AckermannVelocitySetpoint_backwards velocity_ned(::px4_msgs::msg::AckermannVelocitySetpoint::_velocity_ned_type arg)
  {
    msg_.velocity_ned = std::move(arg);
    return Init_AckermannVelocitySetpoint_backwards(msg_);
  }

private:
  ::px4_msgs::msg::AckermannVelocitySetpoint msg_;
};

class Init_AckermannVelocitySetpoint_timestamp
{
public:
  Init_AckermannVelocitySetpoint_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AckermannVelocitySetpoint_velocity_ned timestamp(::px4_msgs::msg::AckermannVelocitySetpoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_AckermannVelocitySetpoint_velocity_ned(msg_);
  }

private:
  ::px4_msgs::msg::AckermannVelocitySetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::AckermannVelocitySetpoint>()
{
  return px4_msgs::msg::builder::Init_AckermannVelocitySetpoint_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__BUILDER_HPP_
