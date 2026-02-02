// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simple_arm_interfaces:srv/GripperControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simple_arm_interfaces/srv/gripper_control.hpp"


#ifndef SIMPLE_ARM_INTERFACES__SRV__DETAIL__GRIPPER_CONTROL__BUILDER_HPP_
#define SIMPLE_ARM_INTERFACES__SRV__DETAIL__GRIPPER_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simple_arm_interfaces/srv/detail/gripper_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simple_arm_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperControl_Request_command
{
public:
  Init_GripperControl_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::simple_arm_interfaces::srv::GripperControl_Request command(::simple_arm_interfaces::srv::GripperControl_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simple_arm_interfaces::srv::GripperControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::simple_arm_interfaces::srv::GripperControl_Request>()
{
  return simple_arm_interfaces::srv::builder::Init_GripperControl_Request_command();
}

}  // namespace simple_arm_interfaces


namespace simple_arm_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperControl_Response_message
{
public:
  explicit Init_GripperControl_Response_message(::simple_arm_interfaces::srv::GripperControl_Response & msg)
  : msg_(msg)
  {}
  ::simple_arm_interfaces::srv::GripperControl_Response message(::simple_arm_interfaces::srv::GripperControl_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simple_arm_interfaces::srv::GripperControl_Response msg_;
};

class Init_GripperControl_Response_success
{
public:
  Init_GripperControl_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperControl_Response_message success(::simple_arm_interfaces::srv::GripperControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GripperControl_Response_message(msg_);
  }

private:
  ::simple_arm_interfaces::srv::GripperControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::simple_arm_interfaces::srv::GripperControl_Response>()
{
  return simple_arm_interfaces::srv::builder::Init_GripperControl_Response_success();
}

}  // namespace simple_arm_interfaces


namespace simple_arm_interfaces
{

namespace srv
{

namespace builder
{

class Init_GripperControl_Event_response
{
public:
  explicit Init_GripperControl_Event_response(::simple_arm_interfaces::srv::GripperControl_Event & msg)
  : msg_(msg)
  {}
  ::simple_arm_interfaces::srv::GripperControl_Event response(::simple_arm_interfaces::srv::GripperControl_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simple_arm_interfaces::srv::GripperControl_Event msg_;
};

class Init_GripperControl_Event_request
{
public:
  explicit Init_GripperControl_Event_request(::simple_arm_interfaces::srv::GripperControl_Event & msg)
  : msg_(msg)
  {}
  Init_GripperControl_Event_response request(::simple_arm_interfaces::srv::GripperControl_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GripperControl_Event_response(msg_);
  }

private:
  ::simple_arm_interfaces::srv::GripperControl_Event msg_;
};

class Init_GripperControl_Event_info
{
public:
  Init_GripperControl_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperControl_Event_request info(::simple_arm_interfaces::srv::GripperControl_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GripperControl_Event_request(msg_);
  }

private:
  ::simple_arm_interfaces::srv::GripperControl_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::simple_arm_interfaces::srv::GripperControl_Event>()
{
  return simple_arm_interfaces::srv::builder::Init_GripperControl_Event_info();
}

}  // namespace simple_arm_interfaces

#endif  // SIMPLE_ARM_INTERFACES__SRV__DETAIL__GRIPPER_CONTROL__BUILDER_HPP_
