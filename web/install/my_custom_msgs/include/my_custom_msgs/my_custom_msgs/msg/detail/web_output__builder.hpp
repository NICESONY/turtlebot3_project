// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_custom_msgs:msg/WebOutput.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__BUILDER_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_custom_msgs/msg/detail/web_output__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_custom_msgs
{

namespace msg
{

namespace builder
{

class Init_WebOutput_batt
{
public:
  explicit Init_WebOutput_batt(::my_custom_msgs::msg::WebOutput & msg)
  : msg_(msg)
  {}
  ::my_custom_msgs::msg::WebOutput batt(::my_custom_msgs::msg::WebOutput::_batt_type arg)
  {
    msg_.batt = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_msgs::msg::WebOutput msg_;
};

class Init_WebOutput_mod
{
public:
  explicit Init_WebOutput_mod(::my_custom_msgs::msg::WebOutput & msg)
  : msg_(msg)
  {}
  Init_WebOutput_batt mod(::my_custom_msgs::msg::WebOutput::_mod_type arg)
  {
    msg_.mod = std::move(arg);
    return Init_WebOutput_batt(msg_);
  }

private:
  ::my_custom_msgs::msg::WebOutput msg_;
};

class Init_WebOutput_id
{
public:
  explicit Init_WebOutput_id(::my_custom_msgs::msg::WebOutput & msg)
  : msg_(msg)
  {}
  Init_WebOutput_mod id(::my_custom_msgs::msg::WebOutput::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_WebOutput_mod(msg_);
  }

private:
  ::my_custom_msgs::msg::WebOutput msg_;
};

class Init_WebOutput_yaw_deg
{
public:
  explicit Init_WebOutput_yaw_deg(::my_custom_msgs::msg::WebOutput & msg)
  : msg_(msg)
  {}
  Init_WebOutput_id yaw_deg(::my_custom_msgs::msg::WebOutput::_yaw_deg_type arg)
  {
    msg_.yaw_deg = std::move(arg);
    return Init_WebOutput_id(msg_);
  }

private:
  ::my_custom_msgs::msg::WebOutput msg_;
};

class Init_WebOutput_y
{
public:
  explicit Init_WebOutput_y(::my_custom_msgs::msg::WebOutput & msg)
  : msg_(msg)
  {}
  Init_WebOutput_yaw_deg y(::my_custom_msgs::msg::WebOutput::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_WebOutput_yaw_deg(msg_);
  }

private:
  ::my_custom_msgs::msg::WebOutput msg_;
};

class Init_WebOutput_x
{
public:
  Init_WebOutput_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WebOutput_y x(::my_custom_msgs::msg::WebOutput::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_WebOutput_y(msg_);
  }

private:
  ::my_custom_msgs::msg::WebOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_msgs::msg::WebOutput>()
{
  return my_custom_msgs::msg::builder::Init_WebOutput_x();
}

}  // namespace my_custom_msgs

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__BUILDER_HPP_
