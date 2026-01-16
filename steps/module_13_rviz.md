# Module 13: RViz2 Visualization 👁️

The final module! **RViz2** is your primary tool for visualizing and debugging ROS 2 systems.

---

## 🎯 What You Will Learn

- Navigate the RViz2 interface
- Add and configure displays for your robot
- Visualize TF transforms
- Create and save RViz2 configurations
- Use RViz2 for debugging common issues

---

## 🖥️ What is RViz2?

**RViz2** (ROS Visualization 2) is a 3D visualization tool that allows you to:

| Feature | Description |
|---------|-------------|
| View robot models | Display URDF/Xacro robots |
| Visualize sensor data | Camera, LiDAR, point clouds |
| See TF transforms | Coordinate frame relationships |
| Debug motion | Joint states, trajectories |
| Interactive markers | Manipulate objects in 3D |

---

## 1️⃣ Launch RViz2

### Standalone

```bash
rviz2
```

### With Robot Description (From Launch)

```bash
ros2 launch simple_arm_description display.launch.py
```

---

## 2️⃣ RViz2 Interface Overview

```
┌─────────────────────────────────────────────────────────────┐
│  File  Panels  Displays  Tools  Help        [toolbar]       │
├──────────────┬──────────────────────────────────────────────┤
│              │                                               │
│   Displays   │              3D Viewport                      │
│    Panel     │                                               │
│              │           (Robot visualization)               │
│ ┌──────────┐ │                                               │
│ │ RobotModel│ │                                               │
│ │ TF        │ │                                               │
│ │ Camera   │ │                                               │
│ └──────────┘ │                                               │
├──────────────┼──────────────────────────────────────────────┤
│    Views     │              Time Panel                       │
└──────────────┴──────────────────────────────────────────────┘
```

### Key Panels

| Panel | Purpose |
|-------|---------|
| **Displays** | Add/configure visualization elements |
| **Views** | Camera control (orbit, FPS, top-down) |
| **Tool Properties** | Configure current tool |
| **Time** | Simulation time, reset |

---

## 3️⃣ Essential Displays

### Global Options

First, set the **Fixed Frame** at the top:

```
Global Options
  └─ Fixed Frame: world   (or base_link)
```

> ⚠️ **Important:** If the fixed frame is wrong, nothing will display correctly!

---

### RobotModel Display

Shows your robot from the URDF.

**Add:** Click **Add** → **rviz_default_plugins** → **RobotModel**

**Configure:**
| Setting | Value |
|---------|-------|
| Description Topic | `/robot_description` |
| Visual Enabled | ✓ |
| Collision Enabled | (optional) |
| TF Prefix | (leave blank) |

---

### TF Display

Shows coordinate frame transforms.

**Add:** Click **Add** → **rviz_default_plugins** → **TF**

**Configure:**
| Setting | Value |
|---------|-------|
| Show Names | ✓ |
| Show Axes | ✓ |
| Show Arrows | ✓ |
| Marker Scale | 0.3 |

**Useful for:**
- Debugging frame relationships
- Verifying joint positions
- Understanding robot kinematics

---

### Image Display

Shows camera images.

**Add:** Click **Add** → **rviz_default_plugins** → **Image**

**Configure:**
| Setting | Value |
|---------|-------|
| Image Topic | `/arm/camera/image_raw` |
| Transport | raw |
| Queue Size | 1 |

---

### Camera Display

Shows camera image as 3D overlay.

**Add:** Click **Add** → **rviz_default_plugins** → **Camera**

**Configure:**
| Setting | Value |
|---------|-------|
| Image Topic | `/arm/camera/image_raw` |
| Visibility | 0.5 (transparency) |

---

### LaserScan Display

Shows LiDAR data.

**Add:** Click **Add** → **rviz_default_plugins** → **LaserScan**

**Configure:**
| Setting | Value |
|---------|-------|
| Topic | `/scan` |
| Size (m) | 0.05 |
| Color Transformer | Intensity |

---

## 4️⃣ View Controls

### Camera Views

| View Type | Description | Use Case |
|-----------|-------------|----------|
| **Orbit** | Rotate around focal point | General viewing |
| **FPS** | First-person shooter style | Navigation |
| **TopDownOrtho** | Top-down orthographic | Floor plans |
| **XYOrbit** | XY plane orbit | 2D navigation |

### Mouse Controls (Orbit View)

| Action | Mouse |
|--------|-------|
| Rotate | Left-click + drag |
| Pan | Middle-click + drag |
| Zoom | Scroll wheel |
| Focus on point | Shift + left-click |

---

## 5️⃣ Save and Load Configurations

### Save Configuration

1. **File** → **Save Config As...**
2. Choose location: `~/ros2_ws/src/simple_arm_description/rviz/arm.rviz`

### Load Configuration

```bash
rviz2 -d ~/ros2_ws/src/simple_arm_description/rviz/arm.rviz
```

### Sample Configuration File

Create `rviz/arm.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Global Options:
    Fixed Frame: world
    Frame Rate: 30
    
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
      
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Axes: true
      Marker Scale: 0.3
      
    - Class: rviz_default_plugins/Image
      Name: Camera
      Enabled: true
      Topic:
        Value: /arm/camera/image_raw
        
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Focal Point:
        X: 0
        Y: 0
        Z: 0.5
```

---

## 6️⃣ Debugging with RViz2

### Problem: Robot Not Visible

**Checklist:**
1. ✅ Fixed Frame matches a frame in your URDF (e.g., `world`, `base_link`)
2. ✅ `/robot_description` topic being published
3. ✅ RobotModel display enabled
4. ✅ No URDF errors (check terminal output)

**Debug commands:**
```bash
# Check robot description is published
ros2 topic echo /robot_description --once

# Check TF frames
ros2 run tf2_tools view_frames

# List available frames
ros2 run tf2_ros tf2_echo world base_link
```

---

### Problem: TF Frames Missing

**Symptoms:** Frames appear but connections are broken

**Solutions:**
```bash
# Generate TF tree visualization
ros2 run tf2_tools view_frames

# View the PDF
evince frames.pdf
```

Check that `robot_state_publisher` and `joint_state_publisher` are running.

---

### Problem: Camera Image Not Showing

**Checklist:**
1. ✅ Topic is being published: `ros2 topic hz /arm/camera/image_raw`
2. ✅ Image display topic matches exactly
3. ✅ ROS-Gazebo bridge running (if using simulation)

---

## 7️⃣ Interactive Markers

For debugging and testing, use **InteractiveMarkers**:

**Add:** Click **Add** → **rviz_default_plugins** → **InteractiveMarkers**

This allows you to:
- Move objects with 6-DOF control
- Test collision detection
- Manually position robot parts

---

## 8️⃣ Launch with RViz2 Configuration

Update your launch file to include RViz2 with saved configuration:

```python
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('simple_arm_description')
    rviz_config = os.path.join(pkg_path, 'rviz', 'arm.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # ... rest of launch file
```

---

## 9️⃣ Useful RViz2 Tools

### Measure Tool

Click **Measure** in toolbar, then click two points to measure distance.

### Publish Point Tool

Click to publish a `geometry_msgs/PointStamped` message with the clicked location.

### 2D Pose Estimate

For navigation: publish an initial pose estimate.

### 2D Goal Pose

For navigation: publish a goal location.

---

## 🚨 Common Troubleshooting

### "Fixed frame [X] does not exist"

**Problem:** The specified fixed frame isn't in the TF tree
**Solution:**
```bash
# List available frames
ros2 run tf2_ros tf2_monitor

# Usually set to: world, base_link, or map
```

### Display appears far from origin

**Problem:** Robot is at wrong scale or position
**Solution:**
- Check URDF units (must be meters)
- Verify spawn position in launch file
- Reset the view: **Views** → **Reset**

### Slow or stuttering display

**Problem:** RViz2 running slowly
**Solution:**
- Reduce TF frame display count
- Lower camera image resolution
- Disable unused displays

---

## 📁 Module File Structure

After completing this module:

```
~/ros2_ws/
└── src/
    └── simple_arm_description/
        ├── CMakeLists.txt
        ├── package.xml
        ├── urdf/
        │   ├── simple_arm.urdf.xacro
        │   ├── simple_arm_gazebo.urdf.xacro
        │   ├── materials.xacro
        │   ├── properties.xacro
        │   ├── ros2_control.xacro
        │   ├── sensors.xacro
        │   └── macros/
        ├── config/
        │   └── arm_controllers.yaml
        ├── worlds/
        │   └── simple_world.sdf
        ├── launch/
        │   ├── display.launch.py
        │   └── gazebo.launch.py
        └── rviz/
            └── arm.rviz                    # NEW: Saved configuration
```

---

## ✅ Summary

In this module, you learned:

- Navigate the **RViz2 interface** and panels
- Add **displays** for robot model, TF, cameras, and sensors
- Configure **view controls** for different perspectives
- **Save and load** RViz2 configurations
- **Debug** common visualization problems
- Use RViz2 **tools** for measurement and interaction

---

## 🎉 Tutorial Complete!

Congratulations! You've completed the ROS 2 Jazzy tutorial series. You now have the skills to:

| Module | Skill Acquired |
|--------|----------------|
| 01 | Experience ROS2 with TurtleSim |
| 02 | Create workspaces and packages |
| 03 | Use CLI for inspecting ROS2 systems |
| 04 | Write your first nodes |
| 05 | Deep dive into topics and messages |
| 06 | Implement services and actions |
| 07 | Create launch files |
| 08 | Write URDF robot descriptions |
| 09 | Use Xacro for modular robots |
| 10 | Simulate robots in Gazebo |
| 11 | Control robots with ros2_control |
| 12 | Integrate camera sensors |
| 13 | Visualize and debug with RViz2 |

### Next Steps

- Explore **Nav2** for autonomous navigation
- Learn **MoveIt2** for motion planning
- Implement **SLAM** for mapping and localization
- Connect to **real hardware** using ros2_control

**Happy Robot Building! 🤖**
