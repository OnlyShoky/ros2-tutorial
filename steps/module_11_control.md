# Module 11: ROS 2 Control ⚙️

Your robot is in Gazebo - now let's make it move! **ROS 2 Control** is the standard framework for controlling robot hardware.

---

## 🎯 What You Will Learn

- Understand the ROS 2 Control architecture
- Add ros2_control configuration to your URDF
- Create controller configuration files (YAML)
- Load and manage controllers
- Send joint commands to your robot arm

---

## 🏗️ ROS 2 Control Architecture

ROS 2 Control separates **hardware** from **control logic**:

```
┌─────────────────────────────────────────────────────────┐
│                   Controller Manager                     │
├─────────────────────────────────────────────────────────┤
│  Joint State      │  Joint Trajectory    │  Position    │
│  Broadcaster      │  Controller          │  Controller  │
├─────────────────────────────────────────────────────────┤
│                   Hardware Interface                     │
├─────────────────────────────────────────────────────────┤
│  Real Robot       │  Gazebo Plugin       │  Mock HW     │
└─────────────────────────────────────────────────────────┘
```

### Key Components

| Component | Purpose |
|-----------|---------|
| **Controller Manager** | Loads, configures, and runs controllers |
| **Controllers** | Compute commands (position, velocity, effort) |
| **Hardware Interface** | Communicates with actual hardware or simulator |
| **Broadcasters** | Publish robot state without controlling |

---

## 1️⃣ Install Required Packages

```bash
sudo apt install ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control \
                 ros-jazzy-controller-manager -y
```

---

## 2️⃣ Add ros2_control to URDF

Add the `<ros2_control>` block to your Xacro file.

### Create: `urdf/ros2_control.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ros2_control Hardware Interface -->
  <ros2_control name="GazeboSimSystem" type="system">
    
    <hardware>
      <!-- Plugin for Gazebo integration -->
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    
    <!-- Shoulder Joint -->
    <joint name="shoulder_joint">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    
    <!-- Elbow Joint -->
    <joint name="elbow_joint">
      <command_interface name="position">
        <param name="min">-2.09</param>
        <param name="max">2.09</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    
    <!-- Wrist Joint -->
    <joint name="wrist_joint">
      <command_interface name="position">
        <param name="min">-3.14</param>
        <param name="max">3.14</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    
  </ros2_control>

  <!-- Gazebo Plugin for ros2_control -->
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control">
      <parameters>$(find simple_arm_description)/config/arm_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
```

### Update Main Xacro File

Add include to `simple_arm_gazebo.urdf.xacro`:

```xml
<!-- Add this line -->
<xacro:include filename="$(find simple_arm_description)/urdf/ros2_control.xacro"/>
```

---

## 3️⃣ Create Controller Configuration

Create `config/arm_controllers.yaml`:

```yaml
# Controller Manager Configuration
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # Controllers to load
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController


# Joint State Broadcaster (publishes /joint_states)
joint_state_broadcaster:
  ros__parameters:
    joints:
      - shoulder_joint
      - elbow_joint
      - wrist_joint


# Joint Trajectory Controller (accepts trajectories)
arm_controller:
  ros__parameters:
    joints:
      - shoulder_joint
      - elbow_joint
      - wrist_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    allow_partial_joints_goal: false
    open_loop_control: true
    
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
```

---

## 4️⃣ Update Launch File

Update `launch/gazebo.launch.py` to spawn controllers:

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    pkg_path = get_package_share_directory('simple_arm_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    xacro_file = os.path.join(pkg_path, 'urdf', 'simple_arm_gazebo.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'simple_world.sdf')
    
    robot_description = Command(['xacro ', xacro_file])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )
    
    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'simple_arm',
            '-topic', 'robot_description',
            '-z', '0.42',
        ],
        output='screen'
    )
    
    # Load Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    
    # Load Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )
    
    # Delay controller spawn until robot is ready
    delay_joint_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )
    
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        delay_joint_broadcaster,
        delay_arm_controller,
        bridge,
    ])
```

---

## 5️⃣ Build and Test

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_description
source install/setup.bash
```

### Launch

```bash
ros2 launch simple_arm_description gazebo.launch.py
```

### Verify Controllers

```bash
# List active controllers
ros2 control list_controllers

# Expected output:
# joint_state_broadcaster  [active]
# arm_controller           [active]
```

### Check Joint States

```bash
ros2 topic echo /joint_states
```

---

## 6️⃣ Send Commands to the Arm

### Using Action Interface

The `arm_controller` provides a `FollowJointTrajectory` action:

```bash
# Check action is available
ros2 action list
# /arm_controller/follow_joint_trajectory
```

### Send a Trajectory from Command Line

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [shoulder_joint, elbow_joint, wrist_joint],
      points: [
        {positions: [0.5, 0.5, 0.0], time_from_start: {sec: 2}},
        {positions: [-0.5, -0.5, 1.0], time_from_start: {sec: 4}},
        {positions: [0.0, 0.0, 0.0], time_from_start: {sec: 6}}
      ]
    }
  }"
```

---

## 7️⃣ Create a Python Controller Script

Create `simple_arm_control/arm_trajectory_client.py`:

```python
#!/usr/bin/env python3
"""
Arm Trajectory Client
Sends trajectories to the robot arm controller
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class ArmTrajectoryClient(Node):
    
    def __init__(self):
        super().__init__('arm_trajectory_client')
        
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server connected!')

    def send_trajectory(self, positions_list, times_list):
        """
        Send a trajectory to the arm.
        
        Args:
            positions_list: List of [shoulder, elbow, wrist] positions
            times_list: List of times (seconds) to reach each position
        """
        
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_joint',
            'elbow_joint', 
            'wrist_joint'
        ]
        
        for positions, time_sec in zip(positions_list, times_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(time_sec))
            goal_msg.trajectory.points.append(point)
        
        self.get_logger().info('Sending trajectory...')
        future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted!')
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            self.get_logger().info('Trajectory completed!')
        else:
            self.get_logger().error('Goal rejected!')


def main(args=None):
    rclpy.init(args=args)
    client = ArmTrajectoryClient()
    
    # Define a pick-and-place motion
    positions = [
        [0.0, 0.5, 0.0],     # Reach forward
        [0.0, 0.8, 0.0],     # Reach down
        [0.0, 0.8, 1.57],    # Rotate wrist
        [0.5, 0.3, 1.57],    # Move to side
        [0.0, 0.0, 0.0],     # Return home
    ]
    times = [2, 3, 4, 6, 8]
    
    client.send_trajectory(positions, times)
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add to `setup.py`:

```python
'arm_trajectory_client = simple_arm_control.arm_trajectory_client:main',
```

---

## 8️⃣ Controller Manager Commands

### List Controllers

```bash
ros2 control list_controllers
```

### List Hardware Interfaces

```bash
ros2 control list_hardware_interfaces
```

### Switch Controllers

```bash
# Deactivate a controller
ros2 control set_controller_state arm_controller inactive

# Activate a controller
ros2 control set_controller_state arm_controller active
```

---

## 🚨 Common Troubleshooting

### Controller Not Found

**Problem:** `spawner` fails with "Could not find controller"
**Solution:**
- Verify YAML file exists at correct path
- Check controller name matches in YAML and launch file

### Joint State Not Publishing

**Problem:** `/joint_states` is empty or not updating
**Solution:**
- Ensure `joint_state_broadcaster` is active
- Verify joint names match between URDF and config

### Action Goal Rejected

**Problem:** Trajectory goal is rejected
**Solution:**
- Check joint limits in URDF
- Verify trajectory joint names match controller config

---

## 📁 Module File Structure

After completing this module:

```
~/ros2_ws/
└── src/
    ├── simple_arm_description/
    │   ├── config/
    │   │   └── arm_controllers.yaml          # NEW
    │   ├── urdf/
    │   │   ├── ros2_control.xacro            # NEW
    │   │   └── simple_arm_gazebo.urdf.xacro  # Updated
    │   └── launch/
    │       └── gazebo.launch.py              # Updated
    │
    └── simple_arm_control/
        └── simple_arm_control/
            └── arm_trajectory_client.py      # NEW
```

---

## ✅ Summary

In this module, you learned:

- **ROS 2 Control** provides a standard way to control robots
- Add `<ros2_control>` blocks to URDF for hardware interface
- **Controllers** are configured via YAML files
- **Joint State Broadcaster** publishes `/joint_states`
- **Joint Trajectory Controller** accepts trajectory goals
- Use **action clients** to send complex trajectories

**Next:** In Module 12, we'll add a **camera sensor** to the robot arm for vision capabilities.
