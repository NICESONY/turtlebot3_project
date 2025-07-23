// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_custom_msgs:msg/WebInput.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__BUILDER_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_custom_msgs/msg/detail/web_input__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_custom_msgs
{

namespace msg
{

namespace builder
{

class Init_WebInput_mod
{
public:
  explicit Init_WebInput_mod(::my_custom_msgs::msg::WebInput & msg)
  : msg_(msg)
  {}
  ::my_custom_msgs::msg::WebInput mod(::my_custom_msgs::msg::WebInput::_mod_type arg)
  {
    msg_.mod = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_custom_msgs::msg::WebInput msg_;
};

class Init_WebInput_id
{
public:
  explicit Init_WebInput_id(::my_custom_msgs::msg::WebInput & msg)
  : msg_(msg)
  {}
  Init_WebInput_mod id(::my_custom_msgs::msg::WebInput::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_WebInput_mod(msg_);
  }

private:
  ::my_custom_msgs::msg::WebInput msg_;
};

class Init_WebInput_yaw_deg
{
public:
  explicit Init_WebInput_yaw_deg(::my_custom_msgs::msg::WebInput & msg)
  : msg_(msg)
  {}
  Init_WebInput_id yaw_deg(::my_custom_msgs::msg::WebInput::_yaw_deg_type arg)
  {
    msg_.yaw_deg = std::move(arg);
    return Init_WebInput_id(msg_);
  }

private:
  ::my_custom_msgs::msg::WebInput msg_;
};

class Init_WebInput_y
{
public:
  explicit Init_WebInput_y(::my_custom_msgs::msg::WebInput & msg)
  : msg_(msg)
  {}
  Init_WebInput_yaw_deg y(::my_custom_msgs::msg::WebInput::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_WebInput_yaw_deg(msg_);
  }

private:
  ::my_custom_msgs::msg::WebInput msg_;
};

class Init_WebInput_x
{
public:
  Init_WebInput_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WebInput_y x(::my_custom_msgs::msg::WebInput::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_WebInput_y(msg_);
  }

private:
  ::my_custom_msgs::msg::WebInput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_custom_msgs::msg::WebInput>()
{
  return my_custom_msgs::msg::builder::Init_WebInput_x();
}

}  // namespace my_custom_msgs

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__BUILDER_HPP_
