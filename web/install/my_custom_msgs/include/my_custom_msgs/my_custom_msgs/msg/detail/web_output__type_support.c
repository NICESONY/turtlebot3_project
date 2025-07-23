// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from my_custom_msgs:msg/WebOutput.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "my_custom_msgs/msg/detail/web_output__rosidl_typesupport_introspection_c.h"
#include "my_custom_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "my_custom_msgs/msg/detail/web_output__functions.h"
#include "my_custom_msgs/msg/detail/web_output__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  my_custom_msgs__msg__WebOutput__init(message_memory);
}

void my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_fini_function(void * message_memory)
{
  my_custom_msgs__msg__WebOutput__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_member_array[6] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__WebOutput, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__WebOutput, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__WebOutput, yaw_deg),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__WebOutput, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "mod",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__WebOutput, mod),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "batt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(my_custom_msgs__msg__WebOutput, batt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_members = {
  "my_custom_msgs__msg",  // message namespace
  "WebOutput",  // message name
  6,  // number of fields
  sizeof(my_custom_msgs__msg__WebOutput),
  my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_member_array,  // message members
  my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_type_support_handle = {
  0,
  &my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_my_custom_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, my_custom_msgs, msg, WebOutput)() {
  if (!my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_type_support_handle.typesupport_identifier) {
    my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &my_custom_msgs__msg__WebOutput__rosidl_typesupport_introspection_c__WebOutput_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
