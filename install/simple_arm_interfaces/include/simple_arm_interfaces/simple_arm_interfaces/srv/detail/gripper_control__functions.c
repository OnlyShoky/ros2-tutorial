// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simple_arm_interfaces:srv/GripperControl.idl
// generated code does not contain a copyright notice
#include "simple_arm_interfaces/srv/detail/gripper_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `command`
#include "rosidl_runtime_c/string_functions.h"

bool
simple_arm_interfaces__srv__GripperControl_Request__init(simple_arm_interfaces__srv__GripperControl_Request * msg)
{
  if (!msg) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__init(&msg->command)) {
    simple_arm_interfaces__srv__GripperControl_Request__fini(msg);
    return false;
  }
  return true;
}

void
simple_arm_interfaces__srv__GripperControl_Request__fini(simple_arm_interfaces__srv__GripperControl_Request * msg)
{
  if (!msg) {
    return;
  }
  // command
  rosidl_runtime_c__String__fini(&msg->command);
}

bool
simple_arm_interfaces__srv__GripperControl_Request__are_equal(const simple_arm_interfaces__srv__GripperControl_Request * lhs, const simple_arm_interfaces__srv__GripperControl_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->command), &(rhs->command)))
  {
    return false;
  }
  return true;
}

bool
simple_arm_interfaces__srv__GripperControl_Request__copy(
  const simple_arm_interfaces__srv__GripperControl_Request * input,
  simple_arm_interfaces__srv__GripperControl_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // command
  if (!rosidl_runtime_c__String__copy(
      &(input->command), &(output->command)))
  {
    return false;
  }
  return true;
}

simple_arm_interfaces__srv__GripperControl_Request *
simple_arm_interfaces__srv__GripperControl_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Request * msg = (simple_arm_interfaces__srv__GripperControl_Request *)allocator.allocate(sizeof(simple_arm_interfaces__srv__GripperControl_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simple_arm_interfaces__srv__GripperControl_Request));
  bool success = simple_arm_interfaces__srv__GripperControl_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simple_arm_interfaces__srv__GripperControl_Request__destroy(simple_arm_interfaces__srv__GripperControl_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simple_arm_interfaces__srv__GripperControl_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simple_arm_interfaces__srv__GripperControl_Request__Sequence__init(simple_arm_interfaces__srv__GripperControl_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Request * data = NULL;

  if (size) {
    data = (simple_arm_interfaces__srv__GripperControl_Request *)allocator.zero_allocate(size, sizeof(simple_arm_interfaces__srv__GripperControl_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simple_arm_interfaces__srv__GripperControl_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simple_arm_interfaces__srv__GripperControl_Request__fini(&data[i - 1]);
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
simple_arm_interfaces__srv__GripperControl_Request__Sequence__fini(simple_arm_interfaces__srv__GripperControl_Request__Sequence * array)
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
      simple_arm_interfaces__srv__GripperControl_Request__fini(&array->data[i]);
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

simple_arm_interfaces__srv__GripperControl_Request__Sequence *
simple_arm_interfaces__srv__GripperControl_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Request__Sequence * array = (simple_arm_interfaces__srv__GripperControl_Request__Sequence *)allocator.allocate(sizeof(simple_arm_interfaces__srv__GripperControl_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simple_arm_interfaces__srv__GripperControl_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simple_arm_interfaces__srv__GripperControl_Request__Sequence__destroy(simple_arm_interfaces__srv__GripperControl_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simple_arm_interfaces__srv__GripperControl_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simple_arm_interfaces__srv__GripperControl_Request__Sequence__are_equal(const simple_arm_interfaces__srv__GripperControl_Request__Sequence * lhs, const simple_arm_interfaces__srv__GripperControl_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simple_arm_interfaces__srv__GripperControl_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simple_arm_interfaces__srv__GripperControl_Request__Sequence__copy(
  const simple_arm_interfaces__srv__GripperControl_Request__Sequence * input,
  simple_arm_interfaces__srv__GripperControl_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simple_arm_interfaces__srv__GripperControl_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simple_arm_interfaces__srv__GripperControl_Request * data =
      (simple_arm_interfaces__srv__GripperControl_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simple_arm_interfaces__srv__GripperControl_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simple_arm_interfaces__srv__GripperControl_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simple_arm_interfaces__srv__GripperControl_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
simple_arm_interfaces__srv__GripperControl_Response__init(simple_arm_interfaces__srv__GripperControl_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    simple_arm_interfaces__srv__GripperControl_Response__fini(msg);
    return false;
  }
  return true;
}

void
simple_arm_interfaces__srv__GripperControl_Response__fini(simple_arm_interfaces__srv__GripperControl_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
simple_arm_interfaces__srv__GripperControl_Response__are_equal(const simple_arm_interfaces__srv__GripperControl_Response * lhs, const simple_arm_interfaces__srv__GripperControl_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
simple_arm_interfaces__srv__GripperControl_Response__copy(
  const simple_arm_interfaces__srv__GripperControl_Response * input,
  simple_arm_interfaces__srv__GripperControl_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

simple_arm_interfaces__srv__GripperControl_Response *
simple_arm_interfaces__srv__GripperControl_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Response * msg = (simple_arm_interfaces__srv__GripperControl_Response *)allocator.allocate(sizeof(simple_arm_interfaces__srv__GripperControl_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simple_arm_interfaces__srv__GripperControl_Response));
  bool success = simple_arm_interfaces__srv__GripperControl_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simple_arm_interfaces__srv__GripperControl_Response__destroy(simple_arm_interfaces__srv__GripperControl_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simple_arm_interfaces__srv__GripperControl_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simple_arm_interfaces__srv__GripperControl_Response__Sequence__init(simple_arm_interfaces__srv__GripperControl_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Response * data = NULL;

  if (size) {
    data = (simple_arm_interfaces__srv__GripperControl_Response *)allocator.zero_allocate(size, sizeof(simple_arm_interfaces__srv__GripperControl_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simple_arm_interfaces__srv__GripperControl_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simple_arm_interfaces__srv__GripperControl_Response__fini(&data[i - 1]);
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
simple_arm_interfaces__srv__GripperControl_Response__Sequence__fini(simple_arm_interfaces__srv__GripperControl_Response__Sequence * array)
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
      simple_arm_interfaces__srv__GripperControl_Response__fini(&array->data[i]);
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

simple_arm_interfaces__srv__GripperControl_Response__Sequence *
simple_arm_interfaces__srv__GripperControl_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Response__Sequence * array = (simple_arm_interfaces__srv__GripperControl_Response__Sequence *)allocator.allocate(sizeof(simple_arm_interfaces__srv__GripperControl_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simple_arm_interfaces__srv__GripperControl_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simple_arm_interfaces__srv__GripperControl_Response__Sequence__destroy(simple_arm_interfaces__srv__GripperControl_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simple_arm_interfaces__srv__GripperControl_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simple_arm_interfaces__srv__GripperControl_Response__Sequence__are_equal(const simple_arm_interfaces__srv__GripperControl_Response__Sequence * lhs, const simple_arm_interfaces__srv__GripperControl_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simple_arm_interfaces__srv__GripperControl_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simple_arm_interfaces__srv__GripperControl_Response__Sequence__copy(
  const simple_arm_interfaces__srv__GripperControl_Response__Sequence * input,
  simple_arm_interfaces__srv__GripperControl_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simple_arm_interfaces__srv__GripperControl_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simple_arm_interfaces__srv__GripperControl_Response * data =
      (simple_arm_interfaces__srv__GripperControl_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simple_arm_interfaces__srv__GripperControl_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simple_arm_interfaces__srv__GripperControl_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simple_arm_interfaces__srv__GripperControl_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "simple_arm_interfaces/srv/detail/gripper_control__functions.h"

bool
simple_arm_interfaces__srv__GripperControl_Event__init(simple_arm_interfaces__srv__GripperControl_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    simple_arm_interfaces__srv__GripperControl_Event__fini(msg);
    return false;
  }
  // request
  if (!simple_arm_interfaces__srv__GripperControl_Request__Sequence__init(&msg->request, 0)) {
    simple_arm_interfaces__srv__GripperControl_Event__fini(msg);
    return false;
  }
  // response
  if (!simple_arm_interfaces__srv__GripperControl_Response__Sequence__init(&msg->response, 0)) {
    simple_arm_interfaces__srv__GripperControl_Event__fini(msg);
    return false;
  }
  return true;
}

void
simple_arm_interfaces__srv__GripperControl_Event__fini(simple_arm_interfaces__srv__GripperControl_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  simple_arm_interfaces__srv__GripperControl_Request__Sequence__fini(&msg->request);
  // response
  simple_arm_interfaces__srv__GripperControl_Response__Sequence__fini(&msg->response);
}

bool
simple_arm_interfaces__srv__GripperControl_Event__are_equal(const simple_arm_interfaces__srv__GripperControl_Event * lhs, const simple_arm_interfaces__srv__GripperControl_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!simple_arm_interfaces__srv__GripperControl_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!simple_arm_interfaces__srv__GripperControl_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
simple_arm_interfaces__srv__GripperControl_Event__copy(
  const simple_arm_interfaces__srv__GripperControl_Event * input,
  simple_arm_interfaces__srv__GripperControl_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!simple_arm_interfaces__srv__GripperControl_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!simple_arm_interfaces__srv__GripperControl_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

simple_arm_interfaces__srv__GripperControl_Event *
simple_arm_interfaces__srv__GripperControl_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Event * msg = (simple_arm_interfaces__srv__GripperControl_Event *)allocator.allocate(sizeof(simple_arm_interfaces__srv__GripperControl_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simple_arm_interfaces__srv__GripperControl_Event));
  bool success = simple_arm_interfaces__srv__GripperControl_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simple_arm_interfaces__srv__GripperControl_Event__destroy(simple_arm_interfaces__srv__GripperControl_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simple_arm_interfaces__srv__GripperControl_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simple_arm_interfaces__srv__GripperControl_Event__Sequence__init(simple_arm_interfaces__srv__GripperControl_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Event * data = NULL;

  if (size) {
    data = (simple_arm_interfaces__srv__GripperControl_Event *)allocator.zero_allocate(size, sizeof(simple_arm_interfaces__srv__GripperControl_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simple_arm_interfaces__srv__GripperControl_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simple_arm_interfaces__srv__GripperControl_Event__fini(&data[i - 1]);
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
simple_arm_interfaces__srv__GripperControl_Event__Sequence__fini(simple_arm_interfaces__srv__GripperControl_Event__Sequence * array)
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
      simple_arm_interfaces__srv__GripperControl_Event__fini(&array->data[i]);
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

simple_arm_interfaces__srv__GripperControl_Event__Sequence *
simple_arm_interfaces__srv__GripperControl_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simple_arm_interfaces__srv__GripperControl_Event__Sequence * array = (simple_arm_interfaces__srv__GripperControl_Event__Sequence *)allocator.allocate(sizeof(simple_arm_interfaces__srv__GripperControl_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simple_arm_interfaces__srv__GripperControl_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simple_arm_interfaces__srv__GripperControl_Event__Sequence__destroy(simple_arm_interfaces__srv__GripperControl_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simple_arm_interfaces__srv__GripperControl_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simple_arm_interfaces__srv__GripperControl_Event__Sequence__are_equal(const simple_arm_interfaces__srv__GripperControl_Event__Sequence * lhs, const simple_arm_interfaces__srv__GripperControl_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simple_arm_interfaces__srv__GripperControl_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simple_arm_interfaces__srv__GripperControl_Event__Sequence__copy(
  const simple_arm_interfaces__srv__GripperControl_Event__Sequence * input,
  simple_arm_interfaces__srv__GripperControl_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simple_arm_interfaces__srv__GripperControl_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simple_arm_interfaces__srv__GripperControl_Event * data =
      (simple_arm_interfaces__srv__GripperControl_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simple_arm_interfaces__srv__GripperControl_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simple_arm_interfaces__srv__GripperControl_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simple_arm_interfaces__srv__GripperControl_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
