
This module introduces **Xacro (XML Macros)**, a powerful tool that makes URDF files shorter, more readable, and easier to maintain.

---

## 🎯 What You Will Learn

- Understand why Xacro improves URDF development
- Use properties for constants and calculations
- Create reusable macros for robot components
- Include and organize files modularly
- Convert between Xacro and URDF

---

## 🤔 Why Use Xacro?

Raw URDF files have several problems:

| Problem | Example |
|---------|---------|
| **Repetition** | Same geometry code for similar links |
| **Magic numbers** | `0.05` scattered throughout the file |
| **No calculations** | Can't compute inertia from dimensions |
| **Single file** | Everything in one massive file |

**Xacro solves all of these!**

---

## 🔧 Xacro Basics

### File Extension

Xacro files use `.urdf.xacro` or `.xacro` extension:

```
robot.urdf.xacro    # Recommended
robot.xacro         # Also valid
```

### XML Namespace

Add the Xacro namespace to your robot tag:

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Xacro content here -->
</robot>
```

---

## 1️⃣ Properties: Named Constants

Properties let you define values once and reuse them.

### Basic Properties

```xml
<!-- Define properties -->
<xacro:property name="arm_length" value="0.5"/>
<xacro:property name="arm_radius" value="0.03"/>
<xacro:property name="arm_mass" value="0.4"/>

<!-- Use properties with ${} -->
<geometry>
  <cylinder radius="${arm_radius}" length="${arm_length}"/>
</geometry>

<mass value="${arm_mass}"/>
```

### Mathematical Expressions

```xml
<!-- Math operations -->
<xacro:property name="half_length" value="${arm_length / 2}"/>
<xacro:property name="total_mass" value="${arm_mass * 3}"/>

<!-- Trigonometry -->
<xacro:property name="pi" value="3.14159265359"/>
<xacro:property name="angle_90" value="${pi / 2}"/>

<!-- Complex calculations -->
<xacro:property name="inertia_xx" value="${(1/12) * arm_mass * (3 * arm_radius**2 + arm_length**2)}"/>
```

### Available Math Functions

| Function | Example |
|----------|---------|
| `+`, `-`, `*`, `/` | `${a + b}` |
| `**` (power) | `${r**2}` |
| `sin`, `cos`, `tan` | `${sin(angle)}` |
| `sqrt`, `abs` | `${sqrt(x)}` |
| `pi` | `${pi}` |

---

## 2️⃣ Macros: Reusable Components

Macros are like functions that generate URDF code.

### Simple Macro

```xml
<!-- Define a macro -->
<xacro:macro name="cylinder_link" params="name length radius color">
  <link name="${name}">
    <visual>
      <origin xyz="0 0 ${length/2}"/>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
      <material name="${color}"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${length/2}"/>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
    </collision>
  </link>
</xacro:macro>

<!-- Use the macro -->
<xacro:cylinder_link name="shoulder" length="0.3" radius="0.05" color="orange"/>
<xacro:cylinder_link name="elbow" length="0.25" radius="0.04" color="blue"/>
<xacro:cylinder_link name="wrist" length="0.1" radius="0.03" color="white"/>
```

### Macro with Default Values

```xml
<xacro:macro name="arm_link" params="name length radius:=0.04 mass:=0.5 color:=gray">
  <!-- Use defaults if not specified -->
</xacro:macro>

<!-- Calls with defaults -->
<xacro:arm_link name="link1" length="0.3"/>
<xacro:arm_link name="link2" length="0.25" color="blue"/>
```

---

## 3️⃣ Including Files

Split your robot into multiple files for organization.

### File Structure

```
urdf/
├── simple_arm.urdf.xacro      # Main file
├── materials.xacro            # Color definitions
├── properties.xacro           # Dimensions and constants
└── macros/
    ├── arm_link.xacro         # Link macro
    └── arm_joint.xacro        # Joint macro
```

### Include Syntax

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include other Xacro files -->
  <xacro:include filename="$(find simple_arm_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find simple_arm_description)/urdf/properties.xacro"/>
  <xacro:include filename="$(find simple_arm_description)/urdf/macros/arm_link.xacro"/>
  
  <!-- Now use included macros and properties -->
  
</robot>
```

---

## 4️⃣ Complete Modular Robot Arm

Let's refactor our robot arm using Xacro.

### Create: `urdf/materials.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Material Definitions -->
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
  
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

</robot>
```

### Create: `urdf/properties.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Mathematical Constants -->
  <xacro:property name="pi" value="3.14159265359"/>
  
  <!-- Base Properties -->
  <xacro:property name="base_radius" value="0.1"/>
  <xacro:property name="base_height" value="0.1"/>
  <xacro:property name="base_mass" value="2.0"/>
  
  <!-- Shoulder Properties -->
  <xacro:property name="shoulder_length" value="0.3"/>
  <xacro:property name="shoulder_radius" value="0.05"/>
  <xacro:property name="shoulder_mass" value="0.5"/>
  
  <!-- Elbow Properties -->
  <xacro:property name="elbow_length" value="0.25"/>
  <xacro:property name="elbow_radius" value="0.04"/>
  <xacro:property name="elbow_mass" value="0.4"/>
  
  <!-- Wrist Properties -->
  <xacro:property name="wrist_length" value="0.1"/>
  <xacro:property name="wrist_radius" value="0.03"/>
  <xacro:property name="wrist_mass" value="0.2"/>
  
  <!-- Joint Limits -->
  <xacro:property name="shoulder_limit" value="${pi/2}"/>
  <xacro:property name="elbow_limit" value="${2*pi/3}"/>
  <xacro:property name="wrist_limit" value="${pi}"/>

</robot>
```

### Create: `urdf/macros/inertia.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro: Cylinder Inertia -->
  <xacro:macro name="cylinder_inertia" params="mass radius length">
    <inertial>
      <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia 
        ixx="${(1/12) * mass * (3 * radius**2 + length**2)}"
        ixy="0"
        ixz="0"
        iyy="${(1/12) * mass * (3 * radius**2 + length**2)}"
        iyz="0"
        izz="${(1/2) * mass * radius**2}"/>
    </inertial>
  </xacro:macro>

  <!-- Macro: Box Inertia -->
  <xacro:macro name="box_inertia" params="mass x y z">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia 
        ixx="${(1/12) * mass * (y**2 + z**2)}"
        ixy="0"
        ixz="0"
        iyy="${(1/12) * mass * (x**2 + z**2)}"
        iyz="0"
        izz="${(1/12) * mass * (x**2 + y**2)}"/>
    </inertial>
  </xacro:macro>

</robot>
```

### Create: `urdf/macros/arm_components.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include inertia macros -->
  <xacro:include filename="$(find simple_arm_description)/urdf/macros/inertia.xacro"/>

  <!-- Macro: Arm Link -->
  <xacro:macro name="arm_link" params="name length radius mass color">
    <link name="${name}_link">
      <visual>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      
      <collision>
        <origin xyz="0 0 ${length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="${radius}" length="${length}"/>
        </geometry>
      </collision>
      
      <xacro:cylinder_inertia mass="${mass}" radius="${radius}" length="${length}"/>
    </link>
  </xacro:macro>

  <!-- Macro: Revolute Joint -->
  <xacro:macro name="arm_joint" params="name parent child xyz axis limit">
    <joint name="${name}_joint" type="revolute">
      <parent link="${parent}_link"/>
      <child link="${child}_link"/>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <axis xyz="${axis}"/>
      <limit lower="${-limit}" upper="${limit}" effort="100" velocity="1.0"/>
      <dynamics damping="0.5"/>
    </joint>
  </xacro:macro>

</robot>
```

### Create: `urdf/simple_arm.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Include all components -->
  <xacro:include filename="$(find simple_arm_description)/urdf/materials.xacro"/>
  <xacro:include filename="$(find simple_arm_description)/urdf/properties.xacro"/>
  <xacro:include filename="$(find simple_arm_description)/urdf/macros/arm_components.xacro"/>

  <!-- World Link -->
  <link name="world"/>

  <!-- Base -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 ${base_height/2}"/>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_height}"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${base_height/2}"/>
      <geometry>
        <cylinder radius="${base_radius}" length="${base_height}"/>
      </geometry>
    </collision>
    <xacro:cylinder_inertia mass="${base_mass}" radius="${base_radius}" length="${base_height}"/>
  </link>

  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

  <!-- Arm Links (using macros!) -->
  <xacro:arm_link name="shoulder" 
                  length="${shoulder_length}" 
                  radius="${shoulder_radius}" 
                  mass="${shoulder_mass}" 
                  color="orange"/>

  <xacro:arm_link name="elbow" 
                  length="${elbow_length}" 
                  radius="${elbow_radius}" 
                  mass="${elbow_mass}" 
                  color="blue"/>

  <xacro:arm_link name="wrist" 
                  length="${wrist_length}" 
                  radius="${wrist_radius}" 
                  mass="${wrist_mass}" 
                  color="white"/>

  <!-- Joints (using macros!) -->
  <xacro:arm_joint name="shoulder" 
                   parent="base" 
                   child="shoulder" 
                   xyz="0 0 ${base_height}" 
                   axis="0 1 0" 
                   limit="${shoulder_limit}"/>

  <xacro:arm_joint name="elbow" 
                   parent="shoulder" 
                   child="elbow" 
                   xyz="0 0 ${shoulder_length}" 
                   axis="0 1 0" 
                   limit="${elbow_limit}"/>

  <xacro:arm_joint name="wrist" 
                   parent="elbow" 
                   child="wrist" 
                   xyz="0 0 ${elbow_length}" 
                   axis="0 0 1" 
                   limit="${wrist_limit}"/>

  <!-- End Effector -->
  <link name="end_effector_link">
    <visual>
      <geometry>
        <box size="0.04 0.06 0.03"/>
      </geometry>
      <material name="gray"/>
    </visual>
  </link>

  <joint name="end_effector_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="end_effector_link"/>
    <origin xyz="0 0 ${wrist_length}"/>
  </joint>

</robot>
```

---

## 5️⃣ Update the Launch File

Update `launch/display.launch.py` to use Xacro:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_path = get_package_share_directory('simple_arm_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'simple_arm.urdf.xacro')
    
    # Process Xacro file
    robot_description = Command(['xacro ', xacro_file])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
```

---

## 6️⃣ Xacro CLI Commands

### Convert Xacro to URDF

```bash
# Generate URDF output
ros2 run xacro xacro ~/ros2_ws/src/simple_arm_description/urdf/simple_arm.urdf.xacro

# Save to file
ros2 run xacro xacro simple_arm.urdf.xacro > simple_arm.urdf

# Check for errors
ros2 run xacro xacro simple_arm.urdf.xacro | check_urdf /dev/stdin
```

### Pass Arguments to Xacro

```bash
ros2 run xacro xacro robot.urdf.xacro arm_length:=0.5 color:=red
```

---

## 🚨 Common Troubleshooting

### Xacro Processing Error

**Problem:** `xacro: error: ...`
**Solution:** 
- Check XML syntax
- Verify `${}` expressions have matching braces
- Ensure included files exist

### Property Not Defined

**Problem:** `Property 'xyz' not defined`
**Solution:** Define property before use or include the file that defines it.

### Macro Not Found

**Problem:** `Macro 'my_macro' not defined`
**Solution:**
- Check macro file is included
- Verify macro name spelling
- Order includes correctly

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
        │   ├── simple_arm.urdf           # Original (Module 06)
        │   ├── simple_arm.urdf.xacro     # NEW: Main Xacro file
        │   ├── materials.xacro           # NEW: Color definitions
        │   ├── properties.xacro          # NEW: Constants
        │   └── macros/
        │       ├── inertia.xacro         # NEW: Inertia calculations
        │       └── arm_components.xacro  # NEW: Link and joint macros
        └── launch/
            └── display.launch.py         # Updated for Xacro
```

---

## ✅ Summary

In this module, you learned:

- **Properties** define reusable constants with calculations
- **Macros** create reusable code blocks with parameters
- **Include** splits files for better organization
- **Default values** make macros more flexible
- **Xacro CLI** converts `.xacro` files to URDF

**Comparison:**

| Approach | simple_arm.urdf | simple_arm.urdf.xacro |
|----------|-----------------|------------------------|
| Lines of code | ~200 | ~50 (main file) |
| Maintainability | Hard | Easy |
| Reusability | None | High |
| Calculations | Manual | Automatic |

**Next:** In Module 08, we'll integrate our robot arm with **Gazebo** for physics simulation.
