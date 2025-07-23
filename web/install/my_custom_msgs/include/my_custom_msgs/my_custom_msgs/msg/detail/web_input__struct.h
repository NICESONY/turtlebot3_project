// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_custom_msgs:msg/WebInput.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__STRUCT_H_
#define MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WebInput in the package my_custom_msgs.
typedef struct my_custom_msgs__msg__WebInput
{
  float x;
  float y;
  float yaw_deg;
  int32_t id;
  int8_t mod;
} my_custom_msgs__msg__WebInput;

// Struct for a sequence of my_custom_msgs__msg__WebInput.
typedef struct my_custom_msgs__msg__WebInput__Sequence
{
  my_custom_msgs__msg__WebInput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_msgs__msg__WebInput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__WEB_INPUT__STRUCT_H_
