
This module introduces the core communication mechanism in ROS 2: **nodes** and **topics**. You'll learn how to create nodes that publish and subscribe to data streams.

---

## 🎯 What You Will Learn

- Understand what nodes are in ROS 2
- Create publisher and subscriber nodes in Python
- Work with topics for data communication
- Use ROS 2 CLI tools to inspect nodes and topics
- Build a joint position system for the robot arm

---

## 🤖 Nodes: The Building Blocks

A **node** is an executable process that performs a specific task. In a robotic system, each node typically handles one function:

| Example Node | Responsibility |
|--------------|----------------|
| `camera_node` | Captures images from camera |
| `arm_controller` | Controls robot arm joints |
| `gripper_node` | Opens/closes the gripper |
| `planner_node` | Plans motion trajectories |

### Benefits of Node-Based Architecture

- **Modularity**: Each node can be developed and tested independently
- **Reusability**: Nodes can be reused across different robots
- **Fault Isolation**: If one node crashes, others continue running
- **Distributed Computing**: Nodes can run on different machines

---

## 📢 Topics: The Communication Channels

**Topics** are named channels for sending messages between nodes. They follow a **publish-subscribe** pattern:

```
┌──────────────┐         ┌──────────────┐
│   Publisher  │ ──────► │    Topic     │ ──────► │  Subscriber  │
│     Node     │         │  /arm/state  │         │     Node     │
└──────────────┘         └──────────────┘         └──────────────┘
```

### Key Characteristics

- **Asynchronous**: Publishers don't wait for subscribers
- **Many-to-Many**: Multiple publishers and subscribers per topic
- **Typed**: Each topic has a specific message type
- **Decoupled**: Publishers don't need to know who subscribes

---

## 1️⃣ Create a Publisher Node

Let's create a node that publishes joint positions for our robot arm.

### Navigate to Your Package

```bash
cd ~/ros2_ws/src/simple_arm_control/simple_arm_control
```

### Create the Publisher File

Create `joint_publisher.py`:

```python
#!/usr/bin/env python3
"""
Joint Position Publisher for Simple Robot Arm
Publishes simulated joint positions to /arm/joint_states
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math


class JointPublisher(Node):
    """
    A ROS 2 node that publishes joint positions for a 3-joint robot arm.
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('joint_publisher')
        
        # Create a publisher
        # - Topic name: /arm/joint_states
        # - Message type: Float64MultiArray (array of floats)
        # - Queue size: 10 (buffer up to 10 messages)
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/arm/joint_states',
            10
        )
        
        # Create a timer that calls publish_joints every 0.1 seconds (10 Hz)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_joints)
        
        # Counter for generating sinusoidal motion
        self.counter = 0.0
        
        self.get_logger().info('Joint Publisher started! Publishing to /arm/joint_states')

    def publish_joints(self):
        """Generate and publish joint positions."""
        
        # Create message
        msg = Float64MultiArray()
        
        # Generate sinusoidal motion for 3 joints
        # This simulates smooth arm movement
        joint1 = math.sin(self.counter) * 0.5              # Shoulder: -0.5 to 0.5 rad
        joint2 = math.sin(self.counter * 0.5) * 0.8        # Elbow: -0.8 to 0.8 rad
        joint3 = math.sin(self.counter * 0.3) * 0.4        # Wrist: -0.4 to 0.4 rad
        
        msg.data = [joint1, joint2, joint3]
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log every 10th message to avoid spam
        if int(self.counter * 10) % 10 == 0:
            self.get_logger().info(
                f'Publishing: [{joint1:.2f}, {joint2:.2f}, {joint3:.2f}]'
            )
        
        self.counter += 0.1


def main(args=None):
    """Main entry point."""
    
    # Initialize ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the node
    node = JointPublisher()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 2️⃣ Create a Subscriber Node

Create `joint_subscriber.py` in the same directory:

```python
#!/usr/bin/env python3
"""
Joint Position Subscriber for Simple Robot Arm
Subscribes to /arm/joint_states and processes the data
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointSubscriber(Node):
    """
    A ROS 2 node that subscribes to joint positions.
    """

    def __init__(self):
        super().__init__('joint_subscriber')
        
        # Create a subscription
        # - Topic: /arm/joint_states
        # - Message type: Float64MultiArray
        # - Callback: joint_callback (called when message received)
        # - Queue size: 10
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/arm/joint_states',
            self.joint_callback,
            10
        )
        
        # Store latest joint positions
        self.joint_positions = [0.0, 0.0, 0.0]
        
        self.get_logger().info('Joint Subscriber started! Listening to /arm/joint_states')

    def joint_callback(self, msg):
        """
        Callback function executed when a message is received.
        
        Args:
            msg: The received Float64MultiArray message
        """
        
        # Extract joint positions from message
        self.joint_positions = list(msg.data)
        
        # Process the data (example: print joint states)
        if len(self.joint_positions) >= 3:
            self.get_logger().info(
                f'Received: Shoulder={self.joint_positions[0]:.2f}, '
                f'Elbow={self.joint_positions[1]:.2f}, '
                f'Wrist={self.joint_positions[2]:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()
    
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

## 3️⃣ Configure Package Entry Points

Edit `setup.py` in your package root to register the nodes:

```python
from setuptools import find_packages, setup

package_name = 'simple_arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Control nodes for simple robot arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Format: 'executable_name = package.module:function'
            'joint_publisher = simple_arm_control.joint_publisher:main',
            'joint_subscriber = simple_arm_control.joint_subscriber:main',
        ],
    },
)
```

---

## 4️⃣ Build and Run

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_control
source install/setup.bash
```

### Run the Publisher (Terminal 1)

```bash
ros2 run simple_arm_control joint_publisher
```

Expected output:
```
[INFO] [joint_publisher]: Joint Publisher started! Publishing to /arm/joint_states
[INFO] [joint_publisher]: Publishing: [0.00, 0.00, 0.00]
[INFO] [joint_publisher]: Publishing: [0.48, 0.25, 0.12]
...
```

### Run the Subscriber (Terminal 2)

Open a new terminal, source the workspace, and run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_arm_control joint_subscriber
```

Expected output:
```
[INFO] [joint_subscriber]: Joint Subscriber started! Listening to /arm/joint_states
[INFO] [joint_subscriber]: Received: Shoulder=0.48, Elbow=0.25, Wrist=0.12
...
```

---

## 5️⃣ Inspect with ROS 2 CLI Tools

### List Active Nodes

```bash
ros2 node list
```

Output:
```
/joint_publisher
/joint_subscriber
```

### List Active Topics

```bash
ros2 topic list
```

Output:
```
/arm/joint_states
/parameter_events
/rosout
```

### Get Topic Info

```bash
ros2 topic info /arm/joint_states
```

Output:
```
Type: std_msgs/msg/Float64MultiArray
Publisher count: 1
Subscriber count: 1
```

### Echo Topic Messages

View messages in real-time:

```bash
ros2 topic echo /arm/joint_states
```

### Measure Publishing Rate

```bash
ros2 topic hz /arm/joint_states
```

Output:
```
average rate: 10.002
        min: 0.099s max: 0.101s std dev: 0.00050s
```

---

## 6️⃣ Node Information

Get detailed information about a node:

```bash
ros2 node info /joint_publisher
```

Output:
```
/joint_publisher
  Subscribers:

  Publishers:
    /arm/joint_states: std_msgs/msg/Float64MultiArray
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    ...
```

---

## 7️⃣ Common Message Types

ROS 2 provides standard message types in the `std_msgs` package:

| Message Type | Description | Example Use |
|--------------|-------------|-------------|
| `String` | Text data | Status messages |
| `Float64` | Single float | Temperature |
| `Float64MultiArray` | Array of floats | Joint positions |
| `Int32` | Integer | Counter |
| `Bool` | True/False | Gripper state |

### View Message Definition

```bash
ros2 interface show std_msgs/msg/Float64MultiArray
```

Output:
```
# This was originally provided as an example message.
# It's not used anywhere and probably shouldn't be used.

std_msgs/MultiArrayLayout layout
float64[] data
```

---

## 🚨 Common Troubleshooting

### Node Not Found

**Problem:** `ros2 run simple_arm_control joint_publisher` fails
**Solution:**
```bash
# Rebuild and source
cd ~/ros2_ws
colcon build --packages-select simple_arm_control
source install/setup.bash
```

### No Messages on Topic

**Problem:** Subscriber runs but receives nothing
**Solution:**
1. Check both nodes are running: `ros2 node list`
2. Verify topic names match exactly (case sensitive)
3. Ensure both nodes are in the same ROS domain: `echo $ROS_DOMAIN_ID`

### Import Error

**Problem:** `ModuleNotFoundError: No module named 'rclpy'`
**Solution:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 📁 Module File Structure

After completing this module, your package should look like:

```
~/ros2_ws/
└── src/
    └── simple_arm_control/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── simple_arm_control
        ├── simple_arm_control/
        │   ├── __init__.py
        │   ├── joint_publisher.py      # NEW
        │   └── joint_subscriber.py     # NEW
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

---

## ✅ Summary

In this module, you learned:

- **Nodes** are independent processes that perform specific tasks
- **Topics** provide publish-subscribe communication between nodes
- Use `self.create_publisher()` to publish messages
- Use `self.create_subscription()` to receive messages
- Configure entry points in `setup.py` for Python packages
- Use `ros2 node` and `ros2 topic` commands for debugging

**Next:** In Module 04, we'll explore **services** and **actions** for request-response and long-running task communication.
