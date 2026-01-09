
This module introduces **URDF (Unified Robot Description Format)**, the standard XML format for describing the physical structure of robots in ROS 2.

---

## 🎯 What You Will Learn

- Understand the URDF file structure
- Define robot links (rigid bodies)
- Create joints to connect links
- Add visual, collision, and inertial properties
- Build a complete 3-joint robot arm URDF
- Visualize your robot in RViz2

---

## 📐 What is URDF?

**URDF** is an XML file that describes:

- **Links**: The rigid parts of your robot (body, arms, wheels)
- **Joints**: How links connect and move relative to each other
- **Visual properties**: How the robot looks
- **Collision properties**: Simplified geometry for physics
- **Inertial properties**: Mass and inertia for dynamics

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links go here -->
  <link name="..."> ... </link>
  
  <!-- Joints go here -->
  <joint name="..." type="..."> ... </joint>
</robot>
```

---

## 🔗 Links: The Building Blocks

A **link** represents a single rigid body in your robot.

### Link Structure

```xml
<link name="link_name">
  
  <!-- Visual: What you see in RViz -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry> ... </geometry>
    <material name="color"/>
  </visual>
  
  <!-- Collision: For physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry> ... </geometry>
  </collision>
  
  <!-- Inertial: Mass and inertia for dynamics -->
  <inertial>
    <origin xyz="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
  
</link>
```

---

## 📦 Geometry Types

URDF supports four geometry types:

### Box

```xml
<geometry>
  <box size="length width height"/>
</geometry>

<!-- Example: 40cm x 20cm x 10cm box -->
<geometry>
  <box size="0.4 0.2 0.1"/>
</geometry>
```

### Cylinder

```xml
<geometry>
  <cylinder radius="0.05" length="0.3"/>
</geometry>
```

### Sphere

```xml
<geometry>
  <sphere radius="0.1"/>
</geometry>
```

### Mesh (External 3D Model)

```xml
<geometry>
  <mesh filename="package://my_package/meshes/gripper.stl"/>
</geometry>
```

---

## 🎨 Materials (Colors)

Define colors for visualization:

```xml
<!-- Define a material -->
<material name="blue">
  <color rgba="0.1 0.1 0.8 1.0"/>  <!-- Red Green Blue Alpha -->
</material>

<!-- Use the material in a link -->
<visual>
  <geometry>
    <box size="0.1 0.1 0.1"/>
  </geometry>
  <material name="blue"/>
</visual>
```

### Common Colors

| Color | RGBA Value |
|-------|------------|
| Red | `0.8 0.1 0.1 1.0` |
| Green | `0.1 0.8 0.1 1.0` |
| Blue | `0.1 0.1 0.8 1.0` |
| Gray | `0.5 0.5 0.5 1.0` |
| Black | `0.1 0.1 0.1 1.0` |
| White | `1.0 1.0 1.0 1.0` |

---

## 🔄 Joints: Connecting Links

A **joint** defines how two links are connected and how they can move relative to each other.

### Joint Structure

```xml
<joint name="joint_name" type="joint_type">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

### Joint Types

| Type | Description | Degrees of Freedom |
|------|-------------|-------------------|
| `fixed` | No movement | 0 |
| `revolute` | Rotation with limits | 1 |
| `continuous` | Unlimited rotation | 1 |
| `prismatic` | Linear sliding with limits | 1 |
| `floating` | 6 DOF (rarely used) | 6 |
| `planar` | Movement in a plane | 3 |

### Origin and Axis

- **Origin**: Position/orientation of child relative to parent
- **Axis**: Direction of movement (for revolute/prismatic)

```xml
<!-- Joint rotates around Y-axis -->
<axis xyz="0 1 0"/>

<!-- Joint rotates around Z-axis -->
<axis xyz="0 0 1"/>
```

---

## 🦾 Building a 3-Joint Robot Arm

Let's create a complete URDF for a simple robot arm.

### Create URDF Directory

```bash
cd ~/ros2_ws/src/simple_arm_description
mkdir -p urdf launch rviz
```

### Create the URDF File

Create `urdf/simple_arm.urdf`:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- ========================================== -->
  <!-- MATERIAL DEFINITIONS                       -->
  <!-- ========================================== -->
  
  <material name="gray">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  
  <material name="orange">
    <color rgba="1.0 0.5 0.0 1.0"/>
  </material>
  
  <material name="blue">
    <color rgba="0.1 0.1 0.8 1.0"/>
  </material>
  
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- ========================================== -->
  <!-- WORLD LINK (Fixed Reference Frame)         -->
  <!-- ========================================== -->
  
  <link name="world"/>

  <!-- ========================================== -->
  <!-- BASE LINK (Stationary Base)                -->
  <!-- ========================================== -->
  
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="gray"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.05"/>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <!-- Fix base to world -->
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- ========================================== -->
  <!-- SHOULDER LINK                              -->
  <!-- ========================================== -->
  
  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="orange"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.15"/>
      <mass value="0.5"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <!-- Shoulder Joint (rotates around Y-axis) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    <dynamics damping="0.5"/>
  </joint>

  <!-- ========================================== -->
  <!-- ELBOW LINK                                 -->
  <!-- ========================================== -->
  
  <link name="elbow_link">
    <visual>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
      <material name="blue"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.125" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.25"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.125"/>
      <mass value="0.4"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.0006"/>
    </inertial>
  </link>
  
  <!-- Elbow Joint (rotates around Y-axis) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.0" upper="2.0" effort="50" velocity="1.5"/>
    <dynamics damping="0.3"/>
  </joint>

  <!-- ========================================== -->
  <!-- WRIST LINK                                 -->
  <!-- ========================================== -->
  
  <link name="wrist_link">
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.03" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.05"/>
      <mass value="0.2"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  
  <!-- Wrist Joint (rotates around Z-axis) -->
  <joint name="wrist_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="2.0"/>
    <dynamics damping="0.1"/>
  </joint>

  <!-- ========================================== -->
  <!-- END EFFECTOR (Tool Mounting Point)         -->
  <!-- ========================================== -->
  
  <link name="end_effector">
    <visual>
      <origin xyz="0 0 0.015" rpy="0 0 0"/>
      <geometry>
        <box size="0.04 0.06 0.03"/>
      </geometry>
      <material name="gray"/>
    </visual>
  </link>
  
  <joint name="end_effector_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

</robot>
```

---

## 🚀 Create a Launch File

Create `launch/display.launch.py`:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package path
    pkg_path = get_package_share_directory('simple_arm_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'simple_arm.urdf')
    
    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher GUI (for testing)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
```

---

## 📦 Update Package Configuration

Update `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_arm_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

Update `package.xml` dependencies:

```xml
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>joint_state_publisher_gui</exec_depend>
<exec_depend>rviz2</exec_depend>
<exec_depend>urdf</exec_depend>
```

---

## 🔨 Build and Run

### Install Dependencies

```bash
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-urdf -y
```

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_description
source install/setup.bash
```

### Launch

```bash
ros2 launch simple_arm_description display.launch.py
```

### Configure RViz2

1. In RViz2, click **Add** in the Displays panel
2. Select **RobotModel** and click OK
3. Set **Description Topic** to `/robot_description`
4. Set **Fixed Frame** to `world`
5. Use the **Joint State Publisher GUI** sliders to move the arm!

---

## 🔍 Validate URDF

Check your URDF for errors:

```bash
# Install check tools
sudo apt install liburdfdom-tools -y

# Check URDF syntax
check_urdf ~/ros2_ws/src/simple_arm_description/urdf/simple_arm.urdf
```

Expected output:
```
robot name is: simple_arm
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  shoulder_link
            child(1):  elbow_link
                child(1):  wrist_link
                    child(1):  end_effector
```

---

## 🚨 Common Troubleshooting

### Robot Not Visible in RViz

**Problem:** RViz shows nothing
**Solution:**
1. Check Fixed Frame is set to `world`
2. Verify Description Topic is `/robot_description`
3. Check for URDF errors: `check_urdf your_robot.urdf`

### TF Tree Broken

**Problem:** Links appear disconnected
**Solution:**
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check for missing links
evince frames.pdf
```

### Joint Doesn't Move

**Problem:** Slider doesn't affect robot
**Solution:** Ensure joint has `type="revolute"` or `type="continuous"`, not `type="fixed"`

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
        │   └── simple_arm.urdf          # NEW: Robot description
        ├── launch/
        │   └── display.launch.py        # NEW: Launch file
        └── rviz/
            └── (optional saved config)
```

---

## ✅ Summary

In this module, you learned:

- **URDF** describes robot structure in XML format
- **Links** are rigid bodies with visual, collision, and inertial properties
- **Joints** connect links and define movement types
- Use **geometry** types: box, cylinder, sphere, mesh
- **Materials** define colors for visualization
- **Launch files** start multiple nodes together
- **RViz2** visualizes your robot model

**Next:** In Module 07, we'll use **Xacro** to make our URDF files more modular and maintainable.
