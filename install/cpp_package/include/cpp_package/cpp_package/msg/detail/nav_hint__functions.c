// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cpp_package:msg/NavHint.idl
// generated code does not contain a copyright notice
#include "cpp_package/msg/detail/nav_hint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `action`
#include "rosidl_runtime_c/string_functions.h"

bool
cpp_package__msg__NavHint__init(cpp_package__msg__NavHint * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cpp_package__msg__NavHint__fini(msg);
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__init(&msg->action)) {
    cpp_package__msg__NavHint__fini(msg);
    return false;
  }
  return true;
}

void
cpp_package__msg__NavHint__fini(cpp_package__msg__NavHint * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // action
  rosidl_runtime_c__String__fini(&msg->action);
}

bool
cpp_package__msg__NavHint__are_equal(const cpp_package__msg__NavHint * lhs, const cpp_package__msg__NavHint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->action), &(rhs->action)))
  {
    return false;
  }
  return true;
}

bool
cpp_package__msg__NavHint__copy(
  const cpp_package__msg__NavHint * input,
  cpp_package__msg__NavHint * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // action
  if (!rosidl_runtime_c__String__copy(
      &(input->action), &(output->action)))
  {
    return false;
  }
  return true;
}

cpp_package__msg__NavHint *
cpp_package__msg__NavHint__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp_package__msg__NavHint * msg = (cpp_package__msg__NavHint *)allocator.allocate(sizeof(cpp_package__msg__NavHint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_package__msg__NavHint));
  bool success = cpp_package__msg__NavHint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cpp_package__msg__NavHint__destroy(cpp_package__msg__NavHint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cpp_package__msg__NavHint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cpp_package__msg__NavHint__Sequence__init(cpp_package__msg__NavHint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp_package__msg__NavHint * data = NULL;

  if (size) {
    data = (cpp_package__msg__NavHint *)allocator.zero_allocate(size, sizeof(cpp_package__msg__NavHint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_package__msg__NavHint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_package__msg__NavHint__fini(&data[i - 1]);
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
cpp_package__msg__NavHint__Sequence__fini(cpp_package__msg__NavHint__Sequence * array)
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
      cpp_package__msg__NavHint__fini(&array->data[i]);
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

cpp_package__msg__NavHint__Sequence *
cpp_package__msg__NavHint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp_package__msg__NavHint__Sequence * array = (cpp_package__msg__NavHint__Sequence *)allocator.allocate(sizeof(cpp_package__msg__NavHint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cpp_package__msg__NavHint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cpp_package__msg__NavHint__Sequence__destroy(cpp_package__msg__NavHint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cpp_package__msg__NavHint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cpp_package__msg__NavHint__Sequence__are_equal(const cpp_package__msg__NavHint__Sequence * lhs, const cpp_package__msg__NavHint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_package__msg__NavHint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_package__msg__NavHint__Sequence__copy(
  const cpp_package__msg__NavHint__Sequence * input,
  cpp_package__msg__NavHint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_package__msg__NavHint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cpp_package__msg__NavHint * data =
      (cpp_package__msg__NavHint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_package__msg__NavHint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cpp_package__msg__NavHint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_package__msg__NavHint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
