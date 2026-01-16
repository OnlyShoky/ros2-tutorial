# Module 03: Exploring ROS2 with CLI Tools 🔍

Remember those commands you used to inspect TurtleSim? Now let's master them. The CLI (Command Line Interface) tools are your eyes and ears in the ROS2 world.

---

## 🎯 What You Will Learn

- Master core `ros2` CLI commands
- Inspect nodes, topics, services, and parameters
- Record and playback data with `ros2 bag`
- Debug common issues with `ros2 doctor`
- Develop efficient debugging workflows

---

## 🔧 The ros2 Command Structure

All ROS 2 CLI commands follow this pattern:

```bash
ros2 <command> <subcommand> [options] [arguments]
```

Get help for any command:

```bash
ros2 --help                    # List all commands
ros2 topic --help              # Help for topic command
ros2 topic echo --help         # Help for specific subcommand
```

---

## 1️⃣ Node Commands

Nodes are the processes running in your system.

### List Running Nodes

```bash
ros2 node list
```

Example output:
```
/joint_publisher
/joint_subscriber
/gripper_server
```

### Get Node Information

```bash
ros2 node info /joint_publisher
```

Output shows:
- Subscriptions
- Publishers
- Service servers
- Service clients
- Action servers/clients

---

## 2️⃣ Topic Commands

Topics are channels for continuous data flow.

### List Active Topics

```bash
ros2 topic list
```

### List with Message Types

```bash
ros2 topic list -t
```

Output:
```
/arm/joint_states [std_msgs/msg/Float64MultiArray]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

### Get Topic Details

```bash
ros2 topic info /arm/joint_states
```

Output:
```
Type: std_msgs/msg/Float64MultiArray
Publisher count: 1
Subscriber count: 1
```

### View Live Messages

```bash
# Echo all messages
ros2 topic echo /arm/joint_states

# Echo only once
ros2 topic echo /arm/joint_states --once

# Echo specific field
ros2 topic echo /arm/joint_states --field data
```

### Measure Topic Statistics

```bash
# Publishing rate (Hz)
ros2 topic hz /arm/joint_states

# Bandwidth usage
ros2 topic bw /arm/joint_states

# Delay between messages
ros2 topic delay /arm/joint_states
```

### Publish from Command Line

```bash
# Publish a single message
ros2 topic pub /arm/joint_states std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.3, 0.1]}"

# Publish at 10 Hz
ros2 topic pub /arm/joint_states std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.3, 0.1]}" --rate 10

# Publish once
ros2 topic pub /arm/joint_states std_msgs/msg/Float64MultiArray \
  "{data: [0.5, 0.3, 0.1]}" --once
```

---

## 3️⃣ Service Commands

Services provide request-response communication.

### List Available Services

```bash
ros2 service list
```

### Get Service Type

```bash
ros2 service type /arm/gripper_control
```

### Call a Service

```bash
ros2 service call /arm/gripper_control simple_arm_interfaces/srv/GripperControl \
  "{command: 'open', position: 0.0}"
```

### Find Services by Type

```bash
ros2 service find simple_arm_interfaces/srv/GripperControl
```

---

## 4️⃣ Parameter Commands

Parameters are runtime configuration values.

### List Node Parameters

```bash
ros2 param list /joint_publisher
```

### Get Parameter Value

```bash
ros2 param get /joint_publisher use_sim_time
```

### Set Parameter Value

```bash
ros2 param set /joint_publisher some_parameter 42
```

### Dump All Parameters to File

```bash
ros2 param dump /joint_publisher > params.yaml
```

### Load Parameters from File

```bash
ros2 param load /joint_publisher params.yaml
```

---

## 5️⃣ Interface Commands

Interfaces define message, service, and action types.

### List All Interfaces

```bash
ros2 interface list
```

### Filter by Type

```bash
ros2 interface list -m    # Messages only
ros2 interface list -s    # Services only
ros2 interface list -a    # Actions only
```

### Show Interface Definition

```bash
# Message type
ros2 interface show std_msgs/msg/Float64MultiArray

# Service type
ros2 interface show simple_arm_interfaces/srv/GripperControl

# Action type
ros2 interface show control_msgs/action/FollowJointTrajectory
```

### List Interfaces in a Package

```bash
ros2 interface package std_msgs
```

---

## 6️⃣ Package Commands

Manage and query ROS 2 packages.

### List Installed Packages

```bash
ros2 pkg list
```

### Find a Specific Package

```bash
ros2 pkg list | grep simple_arm
```

### Get Package Path

```bash
ros2 pkg prefix simple_arm_description
```

### Get Package Information

```bash
ros2 pkg xml simple_arm_description
```

### List Package Executables

```bash
ros2 pkg executables simple_arm_control
```

---

## 7️⃣ Run Commands

Execute nodes from packages.

### Run a Node

```bash
ros2 run <package_name> <executable_name>

# Example
ros2 run simple_arm_control joint_publisher
```

### Run with Arguments

```bash
ros2 run simple_arm_control joint_publisher --ros-args \
  -p publish_rate:=20.0 \
  -r /arm/joint_states:=/robot/joints
```

### Common ROS Arguments

| Argument | Description | Example |
|----------|-------------|---------|
| `-p name:=value` | Set parameter | `-p rate:=10.0` |
| `-r from:=to` | Remap topic/service | `-r /input:=/sensor` |
| `--log-level` | Set logging level | `--log-level debug` |

---

## 8️⃣ Launch Commands

Launch files start multiple nodes together.

### Run a Launch File

```bash
ros2 launch <package_name> <launch_file>

# Example
ros2 launch simple_arm_description display.launch.py
```

### Pass Arguments

```bash
ros2 launch simple_arm_description display.launch.py \
  use_rviz:=true \
  robot_name:=my_arm
```

### List Package Launch Files

```bash
ros2 launch --show-args simple_arm_description display.launch.py
```

---

## 9️⃣ Bag Commands (Data Recording)

Record and playback topic data for debugging and testing.

### Record Topics

```bash
# Record specific topics
ros2 bag record /arm/joint_states /arm/gripper_control

# Record all topics
ros2 bag record -a

# Record with custom output name
ros2 bag record -o my_recording /arm/joint_states
```

### Playback Recording

```bash
# Play at normal speed
ros2 bag play my_recording

# Play at half speed
ros2 bag play my_recording --rate 0.5

# Loop playback
ros2 bag play my_recording --loop
```

### Get Recording Information

```bash
ros2 bag info my_recording
```

Output:
```
Files:             my_recording_0.db3
Bag size:          1.2 MiB
Storage id:        sqlite3
Duration:          10.5s
Start:             Jan 09 2026 18:00:00.000 (1736442000.000)
End:               Jan 09 2026 18:00:10.500 (1736442010.500)
Messages:          1050
Topic information: Topic: /arm/joint_states | Type: std_msgs/msg/Float64MultiArray | Count: 1050
```

---

## 🔟 Doctor Commands (Diagnostics)

Diagnose common ROS 2 configuration issues.

### Run Full Diagnostic

```bash
ros2 doctor
```

### Generate Detailed Report

```bash
ros2 doctor --report
```

### Check Specific Areas

```bash
ros2 doctor --include-warnings
```

### Common Issues Detected

- Environment variables not set correctly
- DDS middleware configuration problems
- Network/multicast issues
- Missing dependencies

---

## 🛠️ Debugging Workflow

### Step 1: Check What's Running

```bash
ros2 node list
ros2 topic list
ros2 service list
```

### Step 2: Verify Connections

```bash
ros2 node info /your_node
ros2 topic info /your_topic
```

### Step 3: Inspect Data Flow

```bash
ros2 topic echo /your_topic
ros2 topic hz /your_topic
```

### Step 4: Test Communications

```bash
# Publish test data
ros2 topic pub /test_topic std_msgs/msg/String "{data: 'Hello'}" --once

# Call test service
ros2 service call /your_service ...
```

### Step 5: Check for Errors

```bash
ros2 doctor
ros2 topic echo /rosout    # View log messages
```

---

## 📊 Quick Reference Table

| Task | Command |
|------|---------|
| List nodes | `ros2 node list` |
| List topics | `ros2 topic list` |
| List services | `ros2 service list` |
| View messages | `ros2 topic echo /topic` |
| Get topic rate | `ros2 topic hz /topic` |
| Publish message | `ros2 topic pub /topic type '{data}'` |
| Call service | `ros2 service call /srv type '{req}'` |
| Get params | `ros2 param list /node` |
| Record data | `ros2 bag record /topic1 /topic2` |
| Play recording | `ros2 bag play recording_name` |
| Diagnose issues | `ros2 doctor` |

---

## 🚨 Common Troubleshooting

### Topics Not Visible Between Machines

**Problem:** Nodes on different machines can't communicate
**Solution:**
```bash
# Ensure same ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Set domain ID (same on all machines)
export ROS_DOMAIN_ID=42
```

### Slow Topic Discovery

**Problem:** Topics take a long time to appear
**Solution:**
```bash
# Restart the ROS 2 daemon
ros2 daemon stop
ros2 daemon start
```

### Command Not Found

**Problem:** `ros2: command not found`
**Solution:**
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 📁 Module File Structure

This module focuses on CLI usage rather than creating new files. Your workspace structure remains:

```
~/ros2_ws/
└── src/
    ├── simple_arm_description/
    │   ├── CMakeLists.txt
    │   └── package.xml
    ├── simple_arm_control/
    │   ├── package.xml
    │   ├── setup.py
    │   └── simple_arm_control/
    │       ├── joint_publisher.py
    │       ├── joint_subscriber.py
    │       ├── gripper_server.py
    │       └── gripper_client.py
    └── simple_arm_interfaces/
        ├── CMakeLists.txt
        ├── package.xml
        └── srv/
            └── GripperControl.srv
```

**Optional recordings created:**
```
~/ros2_ws/
└── my_recording/           # Created by ros2 bag record
    ├── metadata.yaml
    └── my_recording_0.db3
```

---

## ✅ Summary

In this module, you learned:

- **Node commands** to inspect running processes
- **Topic commands** to monitor and publish data
- **Service commands** to call and query services
- **Parameter commands** to configure nodes at runtime
- **Bag commands** to record and playback data
- **Doctor command** to diagnose configuration issues
- A systematic **debugging workflow** for ROS 2 systems

**Next:** In Module 04, you'll write your first ROS2 nodes from scratch!
