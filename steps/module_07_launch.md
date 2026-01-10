# Module 07: Launch Files 🚀

Launch files let you start multiple nodes with a single command. They're essential for any real ROS2 project.

---

## 🎯 What You Will Learn

- Understand why launch files are essential
- Create Python launch files
- Launch multiple nodes together
- Pass parameters and arguments
- Use launch file best practices

---

## 🤔 Why Launch Files?

Without launch files:
```bash
# Terminal 1
ros2 run my_pkg node_1
# Terminal 2
ros2 run my_pkg node_2
# Terminal 3
ros2 run my_pkg node_3
# ...this gets tedious!
```

With launch files:
```bash
# One command, all nodes!
ros2 launch my_pkg my_launch.launch.py
```

---

## 1️⃣ Create a Launch Directory

Launch files live in a `launch/` folder in your package:

```bash
cd ~/ros2_ws/src/my_first_pkg
mkdir launch
```

---

## 2️⃣ Your First Launch File

Create `launch/demo.launch.py`:

```python
#!/usr/bin/env python3
"""
My first launch file!
Launches both the publisher and subscriber together.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description."""
    
    # Create node actions
    publisher_node = Node(
        package='my_first_pkg',
        executable='simple_publisher',
        name='my_publisher',
        output='screen'
    )
    
    subscriber_node = Node(
        package='my_first_pkg',
        executable='simple_subscriber', 
        name='my_subscriber',
        output='screen'
    )
    
    # Return the launch description with all nodes
    return LaunchDescription([
        publisher_node,
        subscriber_node,
    ])
```

---

## 3️⃣ Update Package Configuration

### Edit setup.py

Add the launch files to your setup:

```python
import os
from glob import glob
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
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
    ],
    # ... rest of setup
)
```

### Add Dependencies to package.xml

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

---

## 4️⃣ Build and Run

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash

# Launch both nodes!
ros2 launch my_first_pkg demo.launch.py
```

Both nodes start together! Press `Ctrl+C` to stop all.

---

## 5️⃣ Launch with Parameters

Pass parameters to your nodes:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    publisher_node = Node(
        package='my_first_pkg',
        executable='simple_publisher',
        name='my_publisher',
        output='screen',
        parameters=[
            {'publish_rate': 2.0},
            {'message_prefix': 'Hello from launch!'}
        ]
    )
    
    return LaunchDescription([publisher_node])
```

---

## 6️⃣ Launch Arguments

Make your launch files configurable:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Declare an argument
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Publishing rate in Hz'
    )
    
    publisher_node = Node(
        package='my_first_pkg',
        executable='simple_publisher',
        name='my_publisher',
        output='screen',
        parameters=[
            {'publish_rate': LaunchConfiguration('rate')}
        ]
    )
    
    return LaunchDescription([
        rate_arg,
        publisher_node,
    ])
```

Run with custom argument:
```bash
ros2 launch my_first_pkg demo.launch.py rate:=5.0
```

---

## 7️⃣ Include Other Launch Files

Launch files can include other launch files:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    pkg_path = get_package_share_directory('my_first_pkg')
    
    # Include another launch file
    other_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'other.launch.py')
        ),
        launch_arguments={'arg_name': 'value'}.items()
    )
    
    return LaunchDescription([other_launch])
```

---

## 8️⃣ Remapping Topics

Remap topic names at launch time:

```python
publisher_node = Node(
    package='my_first_pkg',
    executable='simple_publisher',
    name='my_publisher',
    output='screen',
    remappings=[
        ('/my_topic', '/renamed_topic'),
    ]
)
```

---

## 📋 Common Launch Actions

| Action | Purpose |
|--------|---------|
| `Node` | Start a ROS2 node |
| `IncludeLaunchDescription` | Include another launch file |
| `DeclareLaunchArgument` | Declare configurable argument |
| `ExecuteProcess` | Run any command |
| `GroupAction` | Group actions with namespace |
| `TimerAction` | Delay actions |

---

## 🚨 Troubleshooting

### Launch File Not Found

**Problem:** `ros2 launch` can't find your file
**Solution:**
```bash
# Make sure launch directory is installed
colcon build --packages-select my_first_pkg
source install/setup.bash

# Check it's installed
ls ~/ros2_ws/install/my_first_pkg/share/my_first_pkg/launch/
```

### Permission Denied

**Problem:** Launch file can't execute
**Solution:**
```bash
chmod +x launch/demo.launch.py
```

---

## 📁 Module File Structure

After completing this module:

```
~/ros2_ws/
└── src/
    └── my_first_pkg/
        ├── package.xml           # Updated with launch deps
        ├── setup.py              # Updated to install launch files
        ├── launch/
        │   └── demo.launch.py    # YOUR LAUNCH FILE!
        └── my_first_pkg/
            ├── simple_publisher.py
            └── simple_subscriber.py
```

---

## ✅ Summary

In this module, you learned:

- **Launch files** start multiple nodes with one command
- Use `ros2 launch package_name file.launch.py`
- **Parameters** configure nodes at launch time
- **Arguments** make launch files reusable
- **Remappings** redirect topics/services
- **Include** other launch files for modularity

**Next:** In Module 08, we'll start describing our robot with URDF!
