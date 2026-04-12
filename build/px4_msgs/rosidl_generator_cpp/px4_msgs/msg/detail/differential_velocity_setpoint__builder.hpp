// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/DifferentialVelocitySetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_VELOCITY_SETPOINT__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_VELOCITY_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/differential_velocity_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_DifferentialVelocitySetpoint_bearing
{
public:
  explicit Init_DifferentialVelocitySetpoint_bearing(::px4_msgs::msg::DifferentialVelocitySetpoint & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::DifferentialVelocitySetpoint bearing(::px4_msgs::msg::DifferentialVelocitySetpoint::_bearing_type arg)
  {
    msg_.bearing = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::DifferentialVelocitySetpoint msg_;
};

class Init_DifferentialVelocitySetpoint_speed
{
public:
  explicit Init_DifferentialVelocitySetpoint_speed(::px4_msgs::msg::DifferentialVelocitySetpoint & msg)
  : msg_(msg)
  {}
  Init_DifferentialVelocitySetpoint_bearing speed(::px4_msgs::msg::DifferentialVelocitySetpoint::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_DifferentialVelocitySetpoint_bearing(msg_);
  }

private:
  ::px4_msgs::msg::DifferentialVelocitySetpoint msg_;
};

class Init_DifferentialVelocitySetpoint_timestamp
{
public:
  Init_DifferentialVelocitySetpoint_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DifferentialVelocitySetpoint_speed timestamp(::px4_msgs::msg::DifferentialVelocitySetpoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DifferentialVelocitySetpoint_speed(msg_);
  }

private:
  ::px4_msgs::msg::DifferentialVelocitySetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::DifferentialVelocitySetpoint>()
{
  return px4_msgs::msg::builder::Init_DifferentialVelocitySetpoint_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__DIFFERENTIAL_VELOCITY_SETPOINT__BUILDER_HPP_
