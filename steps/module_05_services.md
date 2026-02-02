# Module 05: Services - Request & Response �

In Module 04, you learned about **Topics**, which are great for continuous data streams like sensor readings. But what if you want to ask a question and get a specific answer? Or trigger a one-time action?

That's where **Services** come in!

---

## 🎯 What You Will Learn

- Understand the **Service** communication pattern (Request/Response)
- Create custom **Service Definitions** (`.srv` files)
- Build a **Service Server** to handle requests
- Build a **Service Client** to send requests
- Implement a **Gripper Control** system for our robot arm

---

## 🔄 Topics vs Services

| Feature | Topics | Services |
|---------|--------|----------|
| **Pattern** | Publish-Subscribe | Request-Response |
| **Direction** | One-way (continuous) | Two-way (transactional) |
| **Blocking** | No (Async) | Yes (Client waits for response) |
| **Use Case** | Sensor data, Position updates | "Open Gripper", "Reset Robot", "Get Status" |

```
TOPIC (Continuous Stream):
┌────────────┐           ┌────────────┐
│  Publisher │ ────────► │ Subscriber │
└────────────┘   data     └────────────┘

SERVICE (One-time Call):
┌────────────┐  request  ┌────────────┐
│   Client   │ ────────► │   Server   │
│            │ ◄──────── │            │
└────────────┘  response └────────────┘
```

---

## 1️⃣ Create the Robot Arm Packages

We are starting our **Robot Arm Project**! We need two packages:
1. `simple_arm_interfaces`: For custom message/service definitions
2. `simple_arm_control`: For our Python nodes

### Create Interfaces Package

```bash
cd ~/ros2_ws/src
ros2 pkg create simple_arm_interfaces --build-type ament_cmake --dependencies rosidl_default_generators
```

### Create Control Package

```bash
ros2 pkg create simple_arm_control --build-type ament_python --dependencies rclpy simple_arm_interfaces
```

---

## 2️⃣ Define a Custom Service

We need a service to control the robot's gripper. It should accept a command (like "open") and return success status.

### Create the Service File

1. Create the directory:
```bash
mkdir -p ~/ros2_ws/src/simple_arm_interfaces/srv
```

2. Create `srv/GripperControl.srv`:
```bash
cat > ~/ros2_ws/src/simple_arm_interfaces/srv/GripperControl.srv << 'EOF'
# Request
string command      # "open", "close"
---
# Response
bool success
string message
EOF
```

### Configure CMakeLists.txt

Edit `~/ros2_ws/src/simple_arm_interfaces/CMakeLists.txt` to include:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperControl.srv"
)
```

(Ensure this is before `ament_package()`)

### Configure package.xml

Edit `~/ros2_ws/src/simple_arm_interfaces/package.xml`:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

### Build the Interface

We must build the interface package first so Python can see it.

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_interfaces
source install/setup.bash
```

---

## 3️⃣ Create the Service Server

The **Server** waits for requests and processes them.

Create `~/ros2_ws/src/simple_arm_control/simple_arm_control/gripper_server.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from simple_arm_interfaces.srv import GripperControl

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')
        
        # Create the service
        # - Type: GripperControl
        # - Name: /control_gripper
        # - Callback: self.handle_request
        self.srv = self.create_service(
            GripperControl, 
            'control_gripper', 
            self.handle_request
        )
        self.get_logger().info('Gripper Server Ready! Waiting for commands...')

    def handle_request(self, request, response):
        """Callback function that runs when a request is received"""
        self.get_logger().info(f'Received command: "{request.command}"')
        
        # Process the command
        if request.command == "open":
            response.success = True
            response.message = "Gripper is now OPEN"
        elif request.command == "close":
            response.success = True
            response.message = "Gripper is now CLOSED"
        else:
            response.success = False
            response.message = "Unknown command"
            
        self.get_logger().info(f'Sending response: {response.message}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 4️⃣ Create the Service Client

The **Client** sends requests to the server.

Create `~/ros2_ws/src/simple_arm_control/simple_arm_control/gripper_client.py`:

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from simple_arm_interfaces.srv import GripperControl

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.cli = self.create_client(GripperControl, 'control_gripper')
        
        # Wait for server to start
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gripper Server...')
            
    def send_request(self, command_str):
        req = GripperControl.Request()
        req.command = command_str
        
        # Call service asynchronously
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = GripperClient()
    
    # Get command from terminal args, or default to "open"
    cmd = sys.argv[1] if len(sys.argv) > 1 else "open"
    
    response = client.send_request(cmd)
    client.get_logger().info(f'Result: {response.success}, Message: "{response.message}"')
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 5️⃣ Register Nodes & Build

Update `~/ros2_ws/src/simple_arm_control/setup.py` entry points:

```python
    entry_points={
        'console_scripts': [
            'gripper_server = simple_arm_control.gripper_server:main',
            'gripper_client = simple_arm_control.gripper_client:main',
        ],
    },
```

Build the package:

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_control
source install/setup.bash
```

---

## 6️⃣ Test Your Service

**Terminal 1:** Start the Server
```bash
ros2 run simple_arm_control gripper_server
```

**Terminal 2:** Use the Client
```bash
source ~/ros2_ws/install/setup.bash

# Send commands!
ros2 run simple_arm_control gripper_client open
ros2 run simple_arm_control gripper_client close
ros2 run simple_arm_control gripper_client dance
```

**Terminal 3:** Use CLI Tool
You can also call manually!
```bash
ros2 service call /control_gripper simple_arm_interfaces/srv/GripperControl "{command: 'open'}"
```

---

## 🧠 Why is this better than Topics?

Imagine controlling a gripper with a Topic. You would publish "open" and... **hope** it happens. You wouldn't know when it finished or if it failed.

With a **Service**:
1. You send "open"
2. The code **waits** (blocks)
3. You get back "Success: Gripper Opened"
4. ONLY THEN do you proceed to the next step

This logic is crucial for robot orchestration!

---

## ✅ Summary

In this module, you:
- Created the **Project Packages** (`simple_arm...`)
- Defined a **Custom Service** (`GripperControl.srv`)
- Implemented a **Server** that executes logic and returns a status
- Implemented a **Client** that requests actions
- Validated the transaction using CLI tools

**Next:** In Module 06, we'll handle long-running tasks like "Move Robot Arm" using **Actions**!
