// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_custom_msgs:msg/WebInput.idl
// generated code does not contain a copyright notice
#include "my_custom_msgs/msg/detail/web_input__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_custom_msgs__msg__WebInput__init(my_custom_msgs__msg__WebInput * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // yaw_deg
  // id
  // mod
  return true;
}

void
my_custom_msgs__msg__WebInput__fini(my_custom_msgs__msg__WebInput * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // yaw_deg
  // id
  // mod
}

bool
my_custom_msgs__msg__WebInput__are_equal(const my_custom_msgs__msg__WebInput * lhs, const my_custom_msgs__msg__WebInput * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // yaw_deg
  if (lhs->yaw_deg != rhs->yaw_deg) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // mod
  if (lhs->mod != rhs->mod) {
    return false;
  }
  return true;
}

bool
my_custom_msgs__msg__WebInput__copy(
  const my_custom_msgs__msg__WebInput * input,
  my_custom_msgs__msg__WebInput * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // yaw_deg
  output->yaw_deg = input->yaw_deg;
  // id
  output->id = input->id;
  // mod
  output->mod = input->mod;
  return true;
}

my_custom_msgs__msg__WebInput *
my_custom_msgs__msg__WebInput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_custom_msgs__msg__WebInput * msg = (my_custom_msgs__msg__WebInput *)allocator.allocate(sizeof(my_custom_msgs__msg__WebInput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_custom_msgs__msg__WebInput));
  bool success = my_custom_msgs__msg__WebInput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_custom_msgs__msg__WebInput__destroy(my_custom_msgs__msg__WebInput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_custom_msgs__msg__WebInput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_custom_msgs__msg__WebInput__Sequence__init(my_custom_msgs__msg__WebInput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_custom_msgs__msg__WebInput * data = NULL;

  if (size) {
    data = (my_custom_msgs__msg__WebInput *)allocator.zero_allocate(size, sizeof(my_custom_msgs__msg__WebInput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_custom_msgs__msg__WebInput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_custom_msgs__msg__WebInput__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
my_custom_msgs__msg__WebInput__Sequence__fini(my_custom_msgs__msg__WebInput__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      my_custom_msgs__msg__WebInput__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

my_custom_msgs__msg__WebInput__Sequence *
my_custom_msgs__msg__WebInput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_custom_msgs__msg__WebInput__Sequence * array = (my_custom_msgs__msg__WebInput__Sequence *)allocator.allocate(sizeof(my_custom_msgs__msg__WebInput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_custom_msgs__msg__WebInput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_custom_msgs__msg__WebInput__Sequence__destroy(my_custom_msgs__msg__WebInput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_custom_msgs__msg__WebInput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_custom_msgs__msg__WebInput__Sequence__are_equal(const my_custom_msgs__msg__WebInput__Sequence * lhs, const my_custom_msgs__msg__WebInput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_custom_msgs__msg__WebInput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_custom_msgs__msg__WebInput__Sequence__copy(
  const my_custom_msgs__msg__WebInput__Sequence * input,
  my_custom_msgs__msg__WebInput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_custom_msgs__msg__WebInput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_custom_msgs__msg__WebInput * data =
      (my_custom_msgs__msg__WebInput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_custom_msgs__msg__WebInput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_custom_msgs__msg__WebInput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_custom_msgs__msg__WebInput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
