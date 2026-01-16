# Module 04: Your First ROS2 Node ✍️

Time to write your first ROS2 code! In this module, you'll create a simple publisher node from scratch.

---

## 🎯 What You Will Learn

- Create a Python publisher node
- Create a Python subscriber node
- Understand the basic node structure
- Run and test your nodes
- Experience the "I built this!" moment

---

## 📝 Prerequisites

Make sure you:
1. Completed the TurtleSim demo (Module 01)
2. Have your workspace set up (Module 02)
3. Explored CLI tools (Module 03)

---

## 1️⃣ Create Your Package

Navigate to your workspace and create a package for your first nodes:

```bash
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create my_first_pkg \
  --build-type ament_python \
  --dependencies rclpy std_msgs
```

This creates a package structure ready for Python nodes.

---

## 2️⃣ Write Your First Publisher

Create `~/ros2_ws/src/my_first_pkg/my_first_pkg/simple_publisher.py`:

```python
#!/usr/bin/env python3
"""
My First ROS2 Publisher!
Publishes a counter message every second.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """A simple publisher node."""
    
    def __init__(self):
        # Initialize the node with a name
        super().__init__('simple_publisher')
        
        # Create a publisher
        # - Topic name: /my_topic
        # - Message type: String
        # - Queue size: 10
        self.publisher = self.create_publisher(String, '/my_topic', 10)
        
        # Create a timer (calls callback every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Counter for our messages
        self.counter = 0
        
        self.get_logger().info('Simple Publisher has started!')

    def timer_callback(self):
        """Called every second by the timer."""
        # Create a message
        msg = String()
        msg.data = f'Hello ROS2! Count: {self.counter}'
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Log what we published
        self.get_logger().info(f'Published: "{msg.data}"')
        
        self.counter += 1


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create our node
    node = SimplePublisher()
    
    try:
        # Keep the node running
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

## 3️⃣ Write Your First Subscriber

Create `~/ros2_ws/src/my_first_pkg/my_first_pkg/simple_subscriber.py`:

```python
#!/usr/bin/env python3
"""
My First ROS2 Subscriber!
Listens to messages on /my_topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """A simple subscriber node."""
    
    def __init__(self):
        super().__init__('simple_subscriber')
        
        # Create a subscription
        self.subscription = self.create_subscription(
            String,           # Message type
            '/my_topic',      # Topic name
            self.callback,    # Callback function
            10                # Queue size
        )
        
        self.get_logger().info('Simple Subscriber has started!')

    def callback(self, msg):
        """Called when a message is received."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSubscriber()
    
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

## 4️⃣ Register Your Nodes

Edit `~/ros2_ws/src/my_first_pkg/setup.py` to add entry points:

```python
from setuptools import find_packages, setup

package_name = 'my_first_pkg'

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
    maintainer_email='your@email.com',
    description='My first ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_first_pkg.simple_publisher:main',
            'simple_subscriber = my_first_pkg.simple_subscriber:main',
        ],
    },
)
```

---

## 5️⃣ Build and Run

### Build Your Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash
```

### Run the Publisher (Terminal 1)

```bash
ros2 run my_first_pkg simple_publisher
```

You should see:
```
[INFO] [simple_publisher]: Simple Publisher has started!
[INFO] [simple_publisher]: Published: "Hello ROS2! Count: 0"
[INFO] [simple_publisher]: Published: "Hello ROS2! Count: 1"
...
```

### Run the Subscriber (Terminal 2)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_first_pkg simple_subscriber
```

You should see:
```
[INFO] [simple_subscriber]: Simple Subscriber has started!
[INFO] [simple_subscriber]: I heard: "Hello ROS2! Count: 5"
[INFO] [simple_subscriber]: I heard: "Hello ROS2! Count: 6"
...
```

🎉 **Congratulations! Your two nodes are communicating!**

---

## 6️⃣ Verify with CLI Tools

Use what you learned in Module 03:

```bash
# List running nodes
ros2 node list

# See your topic
ros2 topic list

# Echo the messages
ros2 topic echo /my_topic
```

---

## 🧠 Understanding the Code

### Node Lifecycle

```python
rclpy.init()         # Initialize ROS2 client library
node = MyNode()      # Create your node
rclpy.spin(node)     # Keep node running (process callbacks)
node.destroy_node()  # Clean up
rclpy.shutdown()     # Shutdown ROS2
```

### Publisher Pattern

```python
self.publisher = self.create_publisher(
    String,       # Message type
    '/my_topic',  # Topic name
    10            # Queue size
)
self.publisher.publish(msg)  # Send message
```

### Subscriber Pattern

```python
self.subscription = self.create_subscription(
    String,           # Message type
    '/my_topic',      # Topic name
    self.callback,    # Function to call when message arrives
    10                # Queue size
)
```

---

## 🚨 Troubleshooting

### Node Not Found

**Problem:** `ros2 run my_first_pkg simple_publisher` fails
**Solution:**
```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash
```

### Import Error

**Problem:** `ModuleNotFoundError: No module named 'rclpy'`
**Solution:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 📁 Module File Structure

After completing this module:

```
~/ros2_ws/
└── src/
    └── my_first_pkg/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── my_first_pkg
        ├── my_first_pkg/
        │   ├── __init__.py
        │   ├── simple_publisher.py    # YOUR CODE!
        │   └── simple_subscriber.py   # YOUR CODE!
        └── test/
```

---

## ✅ Summary

In this module, you:

- Created your **first Python package**
- Wrote a **publisher node** that sends messages
- Wrote a **subscriber node** that receives messages
- Saw your nodes **communicate through a topic**
- Used CLI tools to verify the system

**Next:** In Module 05, we'll dive deeper into how topics work and explore different message types.
