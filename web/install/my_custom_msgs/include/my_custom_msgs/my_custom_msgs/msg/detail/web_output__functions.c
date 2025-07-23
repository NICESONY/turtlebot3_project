// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from my_custom_msgs:msg/WebOutput.idl
// generated code does not contain a copyright notice
#include "my_custom_msgs/msg/detail/web_output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
my_custom_msgs__msg__WebOutput__init(my_custom_msgs__msg__WebOutput * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // yaw_deg
  // id
  // mod
  // batt
  return true;
}

void
my_custom_msgs__msg__WebOutput__fini(my_custom_msgs__msg__WebOutput * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // yaw_deg
  // id
  // mod
  // batt
}

bool
my_custom_msgs__msg__WebOutput__are_equal(const my_custom_msgs__msg__WebOutput * lhs, const my_custom_msgs__msg__WebOutput * rhs)
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
  // batt
  if (lhs->batt != rhs->batt) {
    return false;
  }
  return true;
}

bool
my_custom_msgs__msg__WebOutput__copy(
  const my_custom_msgs__msg__WebOutput * input,
  my_custom_msgs__msg__WebOutput * output)
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
  // batt
  output->batt = input->batt;
  return true;
}

my_custom_msgs__msg__WebOutput *
my_custom_msgs__msg__WebOutput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_custom_msgs__msg__WebOutput * msg = (my_custom_msgs__msg__WebOutput *)allocator.allocate(sizeof(my_custom_msgs__msg__WebOutput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(my_custom_msgs__msg__WebOutput));
  bool success = my_custom_msgs__msg__WebOutput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
my_custom_msgs__msg__WebOutput__destroy(my_custom_msgs__msg__WebOutput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    my_custom_msgs__msg__WebOutput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
my_custom_msgs__msg__WebOutput__Sequence__init(my_custom_msgs__msg__WebOutput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_custom_msgs__msg__WebOutput * data = NULL;

  if (size) {
    data = (my_custom_msgs__msg__WebOutput *)allocator.zero_allocate(size, sizeof(my_custom_msgs__msg__WebOutput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = my_custom_msgs__msg__WebOutput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        my_custom_msgs__msg__WebOutput__fini(&data[i - 1]);
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
my_custom_msgs__msg__WebOutput__Sequence__fini(my_custom_msgs__msg__WebOutput__Sequence * array)
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
      my_custom_msgs__msg__WebOutput__fini(&array->data[i]);
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

my_custom_msgs__msg__WebOutput__Sequence *
my_custom_msgs__msg__WebOutput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  my_custom_msgs__msg__WebOutput__Sequence * array = (my_custom_msgs__msg__WebOutput__Sequence *)allocator.allocate(sizeof(my_custom_msgs__msg__WebOutput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = my_custom_msgs__msg__WebOutput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
my_custom_msgs__msg__WebOutput__Sequence__destroy(my_custom_msgs__msg__WebOutput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    my_custom_msgs__msg__WebOutput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
my_custom_msgs__msg__WebOutput__Sequence__are_equal(const my_custom_msgs__msg__WebOutput__Sequence * lhs, const my_custom_msgs__msg__WebOutput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!my_custom_msgs__msg__WebOutput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
my_custom_msgs__msg__WebOutput__Sequence__copy(
  const my_custom_msgs__msg__WebOutput__Sequence * input,
  my_custom_msgs__msg__WebOutput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(my_custom_msgs__msg__WebOutput);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    my_custom_msgs__msg__WebOutput * data =
      (my_custom_msgs__msg__WebOutput *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!my_custom_msgs__msg__WebOutput__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          my_custom_msgs__msg__WebOutput__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!my_custom_msgs__msg__WebOutput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
