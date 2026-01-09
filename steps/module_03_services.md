
This module covers two additional communication patterns in ROS 2: **services** for request-response interactions and **actions** for long-running tasks with feedback.

---

## 🎯 What You Will Learn

- Understand when to use services vs topics vs actions
- Create service servers and clients
- Understand the action communication pattern
- Implement a gripper service for the robot arm
- Choose the right communication pattern for your use case

---

## 🔄 Communication Patterns Overview

ROS 2 provides three main communication patterns:

| Pattern | Direction | Use Case |
|---------|-----------|----------|
| **Topics** | Continuous, async | Sensor data, state updates |
| **Services** | Request → Response | Quick operations, queries |
| **Actions** | Goal → Feedback → Result | Long-running tasks |

### When to Use Each

```
┌─────────────────────────────────────────────────────────────┐
│                    Is it continuous data?                    │
│                           │                                  │
│              Yes ─────────┴────────── No                     │
│               │                        │                     │
│           TOPIC                   Is it quick?               │
│       (sensor data)                    │                     │
│                           Yes ─────────┴────────── No        │
│                            │                        │        │
│                        SERVICE                   ACTION      │
│                    (gripper open)           (move to pose)   │
└─────────────────────────────────────────────────────────────┘
```

---

## 📞 Services: Request-Response

A **service** provides synchronous, one-time communication:

```
┌──────────┐      Request      ┌──────────┐
│  Client  │ ─────────────────►│  Server  │
│          │◄───────────────── │          │
└──────────┘      Response     └──────────┘
```

### Key Characteristics

- **Synchronous**: Client waits for response
- **One-to-one**: Single client, single server
- **No continuous data**: One request, one response
- **Blocking**: Client is blocked until response arrives

---

## 1️⃣ Create a Service Definition

Services use `.srv` files to define request and response types. Let's create a gripper control service.

### Create the Service Package

```bash
cd ~/ros2_ws/src

# Create a package for custom interfaces
ros2 pkg create simple_arm_interfaces \
  --build-type ament_cmake \
  --dependencies rosidl_default_generators
```

### Create the Service File

Create the directory and file:

```bash
mkdir -p ~/ros2_ws/src/simple_arm_interfaces/srv
```

Create `srv/GripperControl.srv`:

```
# Request: Command to send to gripper
string command      # "open", "close", or "set"
float64 position    # 0.0 (closed) to 1.0 (open), used with "set"
---
# Response: Result of the operation
bool success        # True if operation succeeded
string message      # Status message
float64 position    # Current gripper position
```

> 📍 **Note:** The `---` separator divides request (above) from response (below).

### Update CMakeLists.txt

Edit `simple_arm_interfaces/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperControl.srv"
)

ament_package()
```

### Update package.xml

Add to `simple_arm_interfaces/package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>simple_arm_interfaces</name>
  <version>0.0.1</version>
  <description>Custom service and message definitions for simple arm</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  
  <exec_depend>rosidl_default_runtime</exec_depend>
  
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Build the Interface

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_interfaces
source install/setup.bash
```

Verify the service was generated:

```bash
ros2 interface show simple_arm_interfaces/srv/GripperControl
```

---

## 2️⃣ Create a Service Server

Create `gripper_server.py` in `simple_arm_control/simple_arm_control/`:

```python
#!/usr/bin/env python3
"""
Gripper Service Server
Provides a service to control the robot arm gripper
"""

import rclpy
from rclpy.node import Node
from simple_arm_interfaces.srv import GripperControl


class GripperServer(Node):
    """
    Service server that controls the gripper.
    """

    def __init__(self):
        super().__init__('gripper_server')
        
        # Current gripper position (0.0 = closed, 1.0 = open)
        self.gripper_position = 0.0
        
        # Create a service server
        self.service = self.create_service(
            GripperControl,                    # Service type
            '/arm/gripper_control',            # Service name
            self.gripper_callback              # Callback function
        )
        
        self.get_logger().info('Gripper Service Server ready!')
        self.get_logger().info('Service available at: /arm/gripper_control')

    def gripper_callback(self, request, response):
        """
        Handle incoming service requests.
        
        Args:
            request: The service request
            response: The service response (to be filled)
        
        Returns:
            The filled response
        """
        
        self.get_logger().info(f'Received command: {request.command}')
        
        if request.command == 'open':
            self.gripper_position = 1.0
            response.success = True
            response.message = 'Gripper opened successfully'
            
        elif request.command == 'close':
            self.gripper_position = 0.0
            response.success = True
            response.message = 'Gripper closed successfully'
            
        elif request.command == 'set':
            # Validate position range
            if 0.0 <= request.position <= 1.0:
                self.gripper_position = request.position
                response.success = True
                response.message = f'Gripper set to position {request.position:.2f}'
            else:
                response.success = False
                response.message = 'Invalid position. Must be between 0.0 and 1.0'
                
        else:
            response.success = False
            response.message = f'Unknown command: {request.command}'
        
        response.position = self.gripper_position
        
        self.get_logger().info(
            f'Response: success={response.success}, position={response.position:.2f}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 3️⃣ Create a Service Client

Create `gripper_client.py`:

```python
#!/usr/bin/env python3
"""
Gripper Service Client
Sends commands to the gripper service
"""

import sys
import rclpy
from rclpy.node import Node
from simple_arm_interfaces.srv import GripperControl


class GripperClient(Node):
    """
    Service client that sends commands to the gripper.
    """

    def __init__(self):
        super().__init__('gripper_client')
        
        # Create a service client
        self.client = self.create_client(
            GripperControl,
            '/arm/gripper_control'
        )
        
        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')
        
        self.get_logger().info('Connected to gripper service!')

    def send_command(self, command, position=0.0):
        """
        Send a command to the gripper service.
        
        Args:
            command: 'open', 'close', or 'set'
            position: Position value for 'set' command
        
        Returns:
            The service response
        """
        
        # Create request
        request = GripperControl.Request()
        request.command = command
        request.position = position
        
        # Send request and wait for response
        self.get_logger().info(f'Sending command: {command}')
        future = self.client.call_async(request)
        
        # Wait for the response
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    
    client = GripperClient()
    
    # Get command from command line arguments
    if len(sys.argv) < 2:
        command = 'open'
    else:
        command = sys.argv[1]
    
    position = 0.5
    if len(sys.argv) >= 3:
        position = float(sys.argv[2])
    
    # Send the command
    response = client.send_command(command, position)
    
    # Print result
    if response.success:
        print(f'✅ Success: {response.message}')
        print(f'   Gripper position: {response.position:.2f}')
    else:
        print(f'❌ Failed: {response.message}')
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 4️⃣ Update Package Configuration

Add dependencies to `simple_arm_control/package.xml`:

```xml
<depend>simple_arm_interfaces</depend>
```

Update `setup.py` entry points:

```python
entry_points={
    'console_scripts': [
        'joint_publisher = simple_arm_control.joint_publisher:main',
        'joint_subscriber = simple_arm_control.joint_subscriber:main',
        'gripper_server = simple_arm_control.gripper_server:main',
        'gripper_client = simple_arm_control.gripper_client:main',
    ],
},
```

---

## 5️⃣ Test the Service

### Build and Source

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_interfaces simple_arm_control
source install/setup.bash
```

### Run the Server (Terminal 1)

```bash
ros2 run simple_arm_control gripper_server
```

### Call the Service (Terminal 2)

Using the client node:

```bash
ros2 run simple_arm_control gripper_client open
ros2 run simple_arm_control gripper_client close
ros2 run simple_arm_control gripper_client set 0.5
```

Using the command line:

```bash
ros2 service call /arm/gripper_control simple_arm_interfaces/srv/GripperControl \
  "{command: 'open', position: 0.0}"
```

---

## 6️⃣ Service CLI Commands

### List Available Services

```bash
ros2 service list
```

### Get Service Type

```bash
ros2 service type /arm/gripper_control
```

### Call Service from CLI

```bash
ros2 service call /arm/gripper_control simple_arm_interfaces/srv/GripperControl \
  "{command: 'set', position: 0.75}"
```

---

## 🎬 Actions: Long-Running Tasks

**Actions** are designed for tasks that:
- Take a long time to complete
- Need to provide progress feedback
- Should be cancellable

```
┌──────────┐       Goal       ┌──────────┐
│  Client  │ ─────────────────►│  Server  │
│          │◄───────────────── │          │
│          │      Feedback     │          │
│          │◄───────────────── │          │
│          │      Feedback     │          │
│          │◄───────────────── │          │
│          │       Result      │          │
└──────────┘                   └──────────┘
```

### Action Components

| Component | Description |
|-----------|-------------|
| **Goal** | What the action should accomplish |
| **Feedback** | Progress updates during execution |
| **Result** | Final outcome when complete |

### Example Use Cases

- Moving robot arm to a target position
- Navigation to a waypoint
- Picking up an object
- Executing a trajectory

> 💡 **Note:** Creating custom actions follows a similar pattern to services but requires more setup. For robot arm control, the built-in `FollowJointTrajectory` action is commonly used, which we'll explore in Module 09.

---

## 📊 Comparison Summary

| Feature | Topic | Service | Action |
|---------|-------|---------|--------|
| Communication | Pub/Sub | Req/Resp | Goal/Feedback/Result |
| Blocking | No | Yes | No |
| Cancellable | N/A | No | Yes |
| Feedback | Continuous | None | Periodic |
| Use Case | Streaming | Quick queries | Long tasks |

---

## 🚨 Common Troubleshooting

### Service Not Found

**Problem:** Client can't find the service
**Solution:**
```bash
# Check service is running
ros2 service list | grep gripper

# Verify service type
ros2 service type /arm/gripper_control
```

### Import Error for Custom Interface

**Problem:** `ModuleNotFoundError: No module named 'simple_arm_interfaces'`
**Solution:**
```bash
# Rebuild the interfaces package first
cd ~/ros2_ws
colcon build --packages-select simple_arm_interfaces
source install/setup.bash
```

### Service Timeout

**Problem:** Client waits forever for service
**Solution:** Ensure the server node is running before starting the client.

---

## 📁 Module File Structure

After completing this module, your workspace should include:

```
~/ros2_ws/
└── src/
    ├── simple_arm_interfaces/          # NEW: Custom interfaces
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv/
    │       └── GripperControl.srv      # NEW: Service definition
    │
    └── simple_arm_control/
        ├── package.xml                 # Updated with dependencies
        ├── setup.py                    # Updated with entry points
        └── simple_arm_control/
            ├── __init__.py
            ├── joint_publisher.py
            ├── joint_subscriber.py
            ├── gripper_server.py       # NEW
            └── gripper_client.py       # NEW
```

---

## ✅ Summary

In this module, you learned:

- **Services** provide synchronous request-response communication
- Create custom service types with `.srv` files
- Implement **servers** to handle service requests
- Create **clients** to call services
- **Actions** are for long-running, cancellable tasks with feedback
- Choose the right pattern based on your communication needs

**Next:** In Module 05, we'll explore essential **ROS 2 CLI commands** for debugging and introspection.
