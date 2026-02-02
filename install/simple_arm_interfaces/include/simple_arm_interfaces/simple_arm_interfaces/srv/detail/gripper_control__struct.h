// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simple_arm_interfaces:srv/GripperControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simple_arm_interfaces/srv/gripper_control.h"


#ifndef SIMPLE_ARM_INTERFACES__SRV__DETAIL__GRIPPER_CONTROL__STRUCT_H_
#define SIMPLE_ARM_INTERFACES__SRV__DETAIL__GRIPPER_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GripperControl in the package simple_arm_interfaces.
typedef struct simple_arm_interfaces__srv__GripperControl_Request
{
  /// "open", "close"
  rosidl_runtime_c__String command;
} simple_arm_interfaces__srv__GripperControl_Request;

// Struct for a sequence of simple_arm_interfaces__srv__GripperControl_Request.
typedef struct simple_arm_interfaces__srv__GripperControl_Request__Sequence
{
  simple_arm_interfaces__srv__GripperControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simple_arm_interfaces__srv__GripperControl_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GripperControl in the package simple_arm_interfaces.
typedef struct simple_arm_interfaces__srv__GripperControl_Response
{
  bool success;
  rosidl_runtime_c__String message;
} simple_arm_interfaces__srv__GripperControl_Response;

// Struct for a sequence of simple_arm_interfaces__srv__GripperControl_Response.
typedef struct simple_arm_interfaces__srv__GripperControl_Response__Sequence
{
  simple_arm_interfaces__srv__GripperControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simple_arm_interfaces__srv__GripperControl_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  simple_arm_interfaces__srv__GripperControl_Event__request__MAX_SIZE = 1
};
// response
enum
{
  simple_arm_interfaces__srv__GripperControl_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/GripperControl in the package simple_arm_interfaces.
typedef struct simple_arm_interfaces__srv__GripperControl_Event
{
  service_msgs__msg__ServiceEventInfo info;
  simple_arm_interfaces__srv__GripperControl_Request__Sequence request;
  simple_arm_interfaces__srv__GripperControl_Response__Sequence response;
} simple_arm_interfaces__srv__GripperControl_Event;

// Struct for a sequence of simple_arm_interfaces__srv__GripperControl_Event.
typedef struct simple_arm_interfaces__srv__GripperControl_Event__Sequence
{
  simple_arm_interfaces__srv__GripperControl_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simple_arm_interfaces__srv__GripperControl_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMPLE_ARM_INTERFACES__SRV__DETAIL__GRIPPER_CONTROL__STRUCT_H_
