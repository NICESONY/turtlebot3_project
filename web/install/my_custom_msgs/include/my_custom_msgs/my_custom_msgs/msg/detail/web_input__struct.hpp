// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from my_custom_msgs:msg/WebInput.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__STRUCT_HPP_
#define MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__my_custom_msgs__msg__WebInput __attribute__((deprecated))
#else
# define DEPRECATED__my_custom_msgs__msg__WebInput __declspec(deprecated)
#endif

namespace my_custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WebInput_
{
  using Type = WebInput_<ContainerAllocator>;

  explicit WebInput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->yaw_deg = 0.0f;
      this->id = 0l;
      this->mod = 0;
    }
  }

  explicit WebInput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->yaw_deg = 0.0f;
      this->id = 0l;
      this->mod = 0;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _yaw_deg_type =
    float;
  _yaw_deg_type yaw_deg;
  using _id_type =
    int32_t;
  _id_type id;
  using _mod_type =
    int8_t;
  _mod_type mod;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__yaw_deg(
    const float & _arg)
  {
    this->yaw_deg = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__mod(
    const int8_t & _arg)
  {
    this->mod = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    my_custom_msgs::msg::WebInput_<ContainerAllocator> *;
  using ConstRawPtr =
    const my_custom_msgs::msg::WebInput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      my_custom_msgs::msg::WebInput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      my_custom_msgs::msg::WebInput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__my_custom_msgs__msg__WebInput
    std::shared_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__my_custom_msgs__msg__WebInput
    std::shared_ptr<my_custom_msgs::msg::WebInput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WebInput_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->yaw_deg != other.yaw_deg) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->mod != other.mod) {
      return false;
    }
    return true;
  }
  bool operator!=(const WebInput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WebInput_

// alias to use template instance with default allocator
using WebInput =
  my_custom_msgs::msg::WebInput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace my_custom_msgs

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__STRUCT_HPP_
