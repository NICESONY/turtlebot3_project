// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from my_custom_msgs:msg/WebOutput.idl
// generated code does not contain a copyright notice

#ifndef MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__STRUCT_H_
#define MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/WebOutput in the package my_custom_msgs.
typedef struct my_custom_msgs__msg__WebOutput
{
  float x;
  float y;
  float yaw_deg;
  int32_t id;
  int8_t mod;
  int32_t batt;
} my_custom_msgs__msg__WebOutput;

// Struct for a sequence of my_custom_msgs__msg__WebOutput.
typedef struct my_custom_msgs__msg__WebOutput__Sequence
{
  my_custom_msgs__msg__WebOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} my_custom_msgs__msg__WebOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MY_CUSTOM_MSGS__MSG__DETAIL__WEB_OUTPUT__STRUCT_H_
