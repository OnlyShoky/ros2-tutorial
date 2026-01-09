
This module covers integrating your robot arm with **Gazebo**, a powerful physics simulator that works seamlessly with ROS 2.

---

## 🎯 What You Will Learn

- Install and configure Gazebo for ROS 2 Jazzy
- Add Gazebo-specific elements to your URDF
- Create world files for simulation environments
- Spawn your robot in Gazebo
- Create launch files for simulation

---

## 🌐 What is Gazebo?

**Gazebo** is a 3D robotics simulator that provides:

| Feature | Description |
|---------|-------------|
| Physics Engine | Realistic dynamics (gravity, collisions, friction) |
| Sensors | Cameras, LiDAR, IMU, and more |
| Environments | Custom worlds with objects and terrain |
| ROS Integration | Direct communication via topics and services |

> 📍 **Note:** ROS 2 Jazzy uses **Gazebo Harmonic** (the newer "Ignition Gazebo" rebranded). Commands use `gz` instead of `gazebo`.

---

## 1️⃣ Install Gazebo

### Install Gazebo and ROS 2 Packages

```bash
# Install Gazebo Harmonic for ROS 2 Jazzy
sudo apt update
sudo apt install ros-jazzy-ros-gz -y

# Additional useful packages
sudo apt install ros-jazzy-ros-gz-sim \
                 ros-jazzy-ros-gz-bridge \
                 ros-jazzy-ros-gz-image \
                 ros-jazzy-ros-gz-interfaces -y
```

### Verify Installation

```bash
# Check Gazebo version
gz sim --version

# Launch empty world
gz sim empty.sdf
```

---

## 2️⃣ Add Gazebo Elements to URDF

Gazebo requires additional elements in your URDF/Xacro files.

### Gazebo Material Colors

URDF materials don't work in Gazebo. Add `<gazebo>` tags:

```xml
<!-- Add after each link definition -->
<gazebo reference="base_link">
  <material>Gazebo/Gray</material>
</gazebo>

<gazebo reference="shoulder_link">
  <material>Gazebo/Orange</material>
</gazebo>

<gazebo reference="elbow_link">
  <material>Gazebo/Blue</material>
</gazebo>

<gazebo reference="wrist_link">
  <material>Gazebo/White</material>
</gazebo>
```

### Common Gazebo Materials

| Material | Color |
|----------|-------|
| `Gazebo/Red` | Red |
| `Gazebo/Green` | Green |
| `Gazebo/Blue` | Blue |
| `Gazebo/Orange` | Orange |
| `Gazebo/Gray` | Gray |
| `Gazebo/White` | White |
| `Gazebo/Black` | Black |

---

## 3️⃣ Create a Gazebo-Ready Xacro

Create `urdf/simple_arm_gazebo.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include base robot description -->
  <xacro:include filename="$(find simple_arm_description)/urdf/simple_arm.urdf.xacro"/>

  <!-- ========================================== -->
  <!-- GAZEBO MATERIALS                           -->
  <!-- ========================================== -->
  
  <gazebo reference="base_link">
    <material>Gazebo/Gray</material>
  </gazebo>

  <gazebo reference="shoulder_link">
    <material>Gazebo/Orange</material>
  </gazebo>

  <gazebo reference="elbow_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="wrist_link">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="end_effector_link">
    <material>Gazebo/Gray</material>
  </gazebo>

  <!-- ========================================== -->
  <!-- GAZEBO ROS2 CONTROL PLUGIN                -->
  <!-- ========================================== -->
  
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control">
      <parameters>$(find simple_arm_description)/config/arm_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

---

## 4️⃣ Create a World File

Create `worlds/simple_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_world">
    
    <!-- Physics Settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Optional: A table for the robot arm -->
    <model name="table">
      <static>true</static>
      <pose>0 0 0.4 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 0.6 0.02</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 0.6 0.02</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

---

## 5️⃣ Create Gazebo Launch File

Create `launch/gazebo.launch.py`:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package paths
    pkg_path = get_package_share_directory('simple_arm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Files
    xacro_file = os.path.join(pkg_path, 'urdf', 'simple_arm_gazebo.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'simple_world.sdf')
    
    # Process Xacro
    robot_description = Command(['xacro ', xacro_file])
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_arm',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.42',  # On top of table
        ],
        output='screen'
    )
    
    # Bridge for ROS-Gazebo communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge,
    ])
```

---

## 6️⃣ Update Package Configuration

Update `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz config worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

Update `package.xml`:

```xml
<exec_depend>ros_gz_sim</exec_depend>
<exec_depend>ros_gz_bridge</exec_depend>
```

---

## 7️⃣ Build and Run

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_description
source install/setup.bash
```

### Launch Simulation

```bash
ros2 launch simple_arm_description gazebo.launch.py
```

You should see:
1. Gazebo window with your world
2. The robot arm spawned on the table
3. ROS 2 topics available for control

---

## 8️⃣ Verify ROS-Gazebo Bridge

Check that topics are bridged:

```bash
# List all topics
ros2 topic list

# You should see:
# /clock
# /robot_description
# /joint_states (if controller is configured)
```

---

## 🚨 Common Troubleshooting

### Gazebo Doesn't Start

**Problem:** Window doesn't appear
**Solution:**
```bash
# Check for GPU issues
export LIBGL_ALWAYS_SOFTWARE=1
gz sim empty.sdf
```

### Robot Falls Through Ground

**Problem:** Robot doesn't rest on ground
**Solution:**
- Ensure collision elements are defined for all links
- Adjust spawn height (`-z` argument)

### No Materials/Colors in Gazebo

**Problem:** Robot appears gray
**Solution:** Add `<gazebo reference="link_name"><material>...</material></gazebo>` for each link

### Bridge Not Working

**Problem:** ROS topics not updating
**Solution:**
```bash
# Check bridge is running
ros2 node list | grep bridge

# Verify topic types match
ros2 topic info /your_topic
```

---

## 📁 Module File Structure

After completing this module, your package should look like:

```
~/ros2_ws/
└── src/
    └── simple_arm_description/
        ├── CMakeLists.txt
        ├── package.xml
        ├── urdf/
        │   ├── simple_arm.urdf.xacro
        │   ├── simple_arm_gazebo.urdf.xacro    # NEW
        │   ├── materials.xacro
        │   ├── properties.xacro
        │   └── macros/
        │       ├── inertia.xacro
        │       └── arm_components.xacro
        ├── worlds/
        │   └── simple_world.sdf                 # NEW
        ├── launch/
        │   ├── display.launch.py
        │   └── gazebo.launch.py                 # NEW
        └── config/
            └── arm_controllers.yaml             # (Created in Module 09)
```

---

## ✅ Summary

In this module, you learned:

- Install **Gazebo Harmonic** for ROS 2 Jazzy
- Add **Gazebo materials** to URDF with `<gazebo>` tags
- Create **world files** (SDF format) for simulation
- Write **launch files** that start Gazebo and spawn robots
- Use **ros_gz_bridge** to connect ROS 2 and Gazebo

**Next:** In Module 09, we'll add **ROS 2 Control** to actually move the robot arm joints in simulation.
