// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/AckermannVelocitySetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/ackermann_velocity_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AckermannVelocitySetpoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: velocity_ned
  {
    if (msg.velocity_ned.size() == 0) {
      out << "velocity_ned: []";
    } else {
      out << "velocity_ned: [";
      size_t pending_items = msg.velocity_ned.size();
      for (auto item : msg.velocity_ned) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: backwards
  {
    out << "backwards: ";
    rosidl_generator_traits::value_to_yaml(msg.backwards, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AckermannVelocitySetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: velocity_ned
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocity_ned.size() == 0) {
      out << "velocity_ned: []\n";
    } else {
      out << "velocity_ned:\n";
      for (auto item : msg.velocity_ned) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: backwards
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "backwards: ";
    rosidl_generator_traits::value_to_yaml(msg.backwards, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AckermannVelocitySetpoint & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::msg::AckermannVelocitySetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::AckermannVelocitySetpoint & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::AckermannVelocitySetpoint>()
{
  return "px4_msgs::msg::AckermannVelocitySetpoint";
}

template<>
inline const char * name<px4_msgs::msg::AckermannVelocitySetpoint>()
{
  return "px4_msgs/msg/AckermannVelocitySetpoint";
}

template<>
struct has_fixed_size<px4_msgs::msg::AckermannVelocitySetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::AckermannVelocitySetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::AckermannVelocitySetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ACKERMANN_VELOCITY_SETPOINT__TRAITS_HPP_
