// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from my_custom_msgs:msg/WebOutput.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__TRAITS_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "my_custom_msgs/msg/detail/web_output__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace my_custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WebOutput & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: yaw_deg
  {
    out << "yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_deg, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: mod
  {
    out << "mod: ";
    rosidl_generator_traits::value_to_yaml(msg.mod, out);
    out << ", ";
  }

  // member: batt
  {
    out << "batt: ";
    rosidl_generator_traits::value_to_yaml(msg.batt, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WebOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: yaw_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_deg: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_deg, out);
    out << "\n";
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: mod
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mod: ";
    rosidl_generator_traits::value_to_yaml(msg.mod, out);
    out << "\n";
  }

  // member: batt
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "batt: ";
    rosidl_generator_traits::value_to_yaml(msg.batt, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WebOutput & msg, bool use_flow_style = false)
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

}  // namespace my_custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use my_custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const my_custom_msgs::msg::WebOutput & msg,
  std::ostream & out, size_t indentation = 0)
{
  my_custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use my_custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const my_custom_msgs::msg::WebOutput & msg)
{
  return my_custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<my_custom_msgs::msg::WebOutput>()
{
  return "my_custom_msgs::msg::WebOutput";
}

template<>
inline const char * name<my_custom_msgs::msg::WebOutput>()
{
  return "my_custom_msgs/msg/WebOutput";
}

template<>
struct has_fixed_size<my_custom_msgs::msg::WebOutput>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<my_custom_msgs::msg::WebOutput>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<my_custom_msgs::msg::WebOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__TRAITS_HPP_
