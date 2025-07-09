// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from cpp_package:msg/ObstacleFeedback.idl
// generated code does not contain a copyright notice
#include "cpp_package/msg/detail/obstacle_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
cpp_package__msg__ObstacleFeedback__init(cpp_package__msg__ObstacleFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    cpp_package__msg__ObstacleFeedback__fini(msg);
    return false;
  }
  // min_left
  // min_center
  // min_right
  // level_left
  // level_center
  // level_right
  return true;
}

void
cpp_package__msg__ObstacleFeedback__fini(cpp_package__msg__ObstacleFeedback * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // min_left
  // min_center
  // min_right
  // level_left
  // level_center
  // level_right
}

bool
cpp_package__msg__ObstacleFeedback__are_equal(const cpp_package__msg__ObstacleFeedback * lhs, const cpp_package__msg__ObstacleFeedback * rhs)
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
  // min_left
  if (lhs->min_left != rhs->min_left) {
    return false;
  }
  // min_center
  if (lhs->min_center != rhs->min_center) {
    return false;
  }
  // min_right
  if (lhs->min_right != rhs->min_right) {
    return false;
  }
  // level_left
  if (lhs->level_left != rhs->level_left) {
    return false;
  }
  // level_center
  if (lhs->level_center != rhs->level_center) {
    return false;
  }
  // level_right
  if (lhs->level_right != rhs->level_right) {
    return false;
  }
  return true;
}

bool
cpp_package__msg__ObstacleFeedback__copy(
  const cpp_package__msg__ObstacleFeedback * input,
  cpp_package__msg__ObstacleFeedback * output)
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
  // min_left
  output->min_left = input->min_left;
  // min_center
  output->min_center = input->min_center;
  // min_right
  output->min_right = input->min_right;
  // level_left
  output->level_left = input->level_left;
  // level_center
  output->level_center = input->level_center;
  // level_right
  output->level_right = input->level_right;
  return true;
}

cpp_package__msg__ObstacleFeedback *
cpp_package__msg__ObstacleFeedback__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp_package__msg__ObstacleFeedback * msg = (cpp_package__msg__ObstacleFeedback *)allocator.allocate(sizeof(cpp_package__msg__ObstacleFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(cpp_package__msg__ObstacleFeedback));
  bool success = cpp_package__msg__ObstacleFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
cpp_package__msg__ObstacleFeedback__destroy(cpp_package__msg__ObstacleFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    cpp_package__msg__ObstacleFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
cpp_package__msg__ObstacleFeedback__Sequence__init(cpp_package__msg__ObstacleFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp_package__msg__ObstacleFeedback * data = NULL;

  if (size) {
    data = (cpp_package__msg__ObstacleFeedback *)allocator.zero_allocate(size, sizeof(cpp_package__msg__ObstacleFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = cpp_package__msg__ObstacleFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        cpp_package__msg__ObstacleFeedback__fini(&data[i - 1]);
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
cpp_package__msg__ObstacleFeedback__Sequence__fini(cpp_package__msg__ObstacleFeedback__Sequence * array)
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
      cpp_package__msg__ObstacleFeedback__fini(&array->data[i]);
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

cpp_package__msg__ObstacleFeedback__Sequence *
cpp_package__msg__ObstacleFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  cpp_package__msg__ObstacleFeedback__Sequence * array = (cpp_package__msg__ObstacleFeedback__Sequence *)allocator.allocate(sizeof(cpp_package__msg__ObstacleFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = cpp_package__msg__ObstacleFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
cpp_package__msg__ObstacleFeedback__Sequence__destroy(cpp_package__msg__ObstacleFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    cpp_package__msg__ObstacleFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
cpp_package__msg__ObstacleFeedback__Sequence__are_equal(const cpp_package__msg__ObstacleFeedback__Sequence * lhs, const cpp_package__msg__ObstacleFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!cpp_package__msg__ObstacleFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
cpp_package__msg__ObstacleFeedback__Sequence__copy(
  const cpp_package__msg__ObstacleFeedback__Sequence * input,
  cpp_package__msg__ObstacleFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(cpp_package__msg__ObstacleFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    cpp_package__msg__ObstacleFeedback * data =
      (cpp_package__msg__ObstacleFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!cpp_package__msg__ObstacleFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          cpp_package__msg__ObstacleFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!cpp_package__msg__ObstacleFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
