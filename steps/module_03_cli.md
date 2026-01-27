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

## ⚠️ Before You Begin: Start TurtleSim!

> 💡 **Important:** All CLI commands in this module inspect **running nodes**. If no nodes are running, commands like `ros2 node list` will return empty results!

Open **three terminals** and start TurtleSim:

**Terminal 1 - Start the simulation:**
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2 - Start keyboard control:**
```bash
ros2 run turtlesim turtle_teleop_key
```

**Terminal 3 - Your CLI exploration terminal:**
```bash
# Use this terminal for all the commands below!
```

🎉 **Now you have real nodes and topics to explore!**

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

Output (with TurtleSim running):
```
/teleop_turtle
/turtlesim
```

> 💡 You see exactly two nodes - the ones you started!

### Get Node Information

```bash
ros2 node info /turtlesim
```

Output shows:
- Subscriptions (topics the node listens to)
- Publishers (topics the node sends data on)
- Service servers (services the node provides)
- Service clients (services the node calls)
- Action servers/clients

Example output:
```
/turtlesim
  Subscribers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    ...
```

---

## 2️⃣ Topic Commands

Topics are channels for continuous data flow.

### List Active Topics

```bash
ros2 topic list
```

Output:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

### List with Message Types

```bash
ros2 topic list -t
```

Output:
```
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]
```

### Get Topic Details

```bash
ros2 topic info /turtle1/cmd_vel
```

Output:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

### View Live Messages

```bash
# Echo all messages (press arrow keys in teleop terminal!)
ros2 topic echo /turtle1/cmd_vel

# Echo only once
ros2 topic echo /turtle1/pose --once

# Echo specific field
ros2 topic echo /turtle1/pose --field x
```

> 🔍 **Try it:** Press the arrow keys in the teleop terminal and watch the messages appear!

### Measure Topic Statistics

```bash
# Publishing rate (Hz)
ros2 topic hz /turtle1/pose

# Bandwidth usage
ros2 topic bw /turtle1/pose

# Delay between messages
ros2 topic delay /turtle1/pose
```

### Publish from Command Line

```bash
# Make the turtle move in a circle!
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# Publish once (single command)
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Publish at 10 Hz
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --rate 10
```

Press `Ctrl+C` to stop publishing.

---

## 3️⃣ Service Commands

Services provide request-response communication.

### List Available Services

```bash
ros2 service list
```

Output:
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
/turtlesim/describe_parameters
...
```

### Get Service Type

```bash
ros2 service type /spawn
```

Output:
```
turtlesim/srv/Spawn
```

### Show Service Definition

```bash
ros2 interface show turtlesim/srv/Spawn
```

Output:
```
float32 x
float32 y
float32 theta
string name # Optional
---
string name
```

### Call a Service

```bash
# Spawn a new turtle!
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"

# Clear the drawing
ros2 service call /clear std_srvs/srv/Empty

# Change pen color (RGB values 0-255, width in pixels)
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen \
"{r: 255, g: 0, b: 0, width: 5, 'off': false}"
```

> 🎨 **Try it:** Change the pen color, then move the turtle to see a colored trail!

### Find Services by Type

```bash
ros2 service find turtlesim/srv/Spawn
```

---

## 4️⃣ Parameter Commands

Parameters are runtime configuration values.

### List Node Parameters

```bash
ros2 param list /turtlesim
```

Output:
```
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
```

### Get Parameter Value

```bash
ros2 param get /turtlesim background_r
```

Output:
```
Integer value is: 69
```

### Set Parameter Value

```bash
# Change background to red!
ros2 param set /turtlesim background_r 255
ros2 param set /turtlesim background_g 0
ros2 param set /turtlesim background_b 0
```

> 🖼️ **Watch the TurtleSim window change color instantly!**

### Dump All Parameters to File

```bash
ros2 param dump /turtlesim > turtlesim_params.yaml
```

### Load Parameters from File

```bash
ros2 param load /turtlesim turtlesim_params.yaml
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
ros2 interface show geometry_msgs/msg/Twist

# Service type
ros2 interface show turtlesim/srv/Spawn

# Action type
ros2 interface show turtlesim/action/RotateAbsolute
```

### List Interfaces in a Package

```bash
ros2 interface package turtlesim
```

Output:
```
turtlesim/srv/SetPen
turtlesim/srv/Spawn
turtlesim/srv/Kill
turtlesim/srv/TeleportAbsolute
turtlesim/srv/TeleportRelative
turtlesim/msg/Color
turtlesim/msg/Pose
turtlesim/action/RotateAbsolute
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
ros2 pkg list | grep turtlesim
```

### Get Package Path

```bash
ros2 pkg prefix turtlesim
```

### Get Package Information

```bash
ros2 pkg xml turtlesim
```

### List Package Executables

```bash
ros2 pkg executables turtlesim
```

Output:
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

---

## 7️⃣ Run Commands

Execute nodes from packages.

### Run a Node

```bash
ros2 run <package_name> <executable_name>

# Example (you already did this!)
ros2 run turtlesim turtlesim_node
```

### Run with Arguments

```bash
ros2 run turtlesim turtlesim_node --ros-args \
  -p background_r:=255 \
  -p background_g:=255 \
  -p background_b:=0
```

### Common ROS Arguments

| Argument | Description | Example |
|----------|-------------|---------|
| `-p name:=value` | Set parameter | `-p background_r:=255` |
| `-r from:=to` | Remap topic/service | `-r /turtle1/cmd_vel:=/cmd_vel` |
| `--log-level` | Set logging level | `--log-level debug` |

---

## 8️⃣ Launch Commands

Launch files start multiple nodes together (covered in detail in Module 07).

### Run a Launch File

```bash
ros2 launch <package_name> <launch_file>

# Example
ros2 launch turtlesim multisim.launch.py
```

### Show Launch Arguments

```bash
ros2 launch turtlesim multisim.launch.py --show-args
```

---

## 9️⃣ Bag Commands (Data Recording)

Record and playback topic data for debugging and testing.

### Record Topics

```bash
# Record specific topics
ros2 bag record /turtle1/cmd_vel /turtle1/pose

# Record all topics
ros2 bag record -a

# Record with custom output name
ros2 bag record -o my_turtle_recording /turtle1/cmd_vel /turtle1/pose
```

> 🎮 **Try it:** Start recording, move the turtle around with keyboard, then stop with `Ctrl+C`.

### Get Recording Information

```bash
ros2 bag info my_turtle_recording
```

Output:
```
Files:             my_turtle_recording_0.db3
Bag size:          24.0 KiB
Storage id:        sqlite3
Duration:          10.5s
Start:             Jan 22 2026 21:00:00.000
End:               Jan 22 2026 21:00:10.500
Messages:          630
Topic information: 
    Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 105
    Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 525
```

### Playback Recording

```bash
# First, close teleop (so it doesn't interfere)
# Then play at normal speed
ros2 bag play my_turtle_recording

# Play at half speed
ros2 bag play my_turtle_recording --rate 0.5

# Loop playback
ros2 bag play my_turtle_recording --loop
```

> 🔄 **Watch the turtle replay your exact movements!**

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

When something isn't working, follow this systematic approach:

### Step 1: Check What's Running

```bash
ros2 node list
ros2 topic list
ros2 service list
```

### Step 2: Verify Connections

```bash
ros2 node info /turtlesim
ros2 topic info /turtle1/cmd_vel
```

### Step 3: Inspect Data Flow

```bash
ros2 topic echo /turtle1/cmd_vel
ros2 topic hz /turtle1/pose
```

### Step 4: Test Communications

```bash
# Publish test data
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}}" --once

# Call test service
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0}"
```

### Step 5: Check for Errors

```bash
ros2 doctor
ros2 topic echo /rosout    # View log messages from all nodes
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

### No Output from `ros2 node list`

**Problem:** Command returns empty
**Solution:** No nodes are running! Start some nodes first:
```bash
ros2 run turtlesim turtlesim_node
```

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

This module focuses on CLI usage rather than creating new files. You may optionally create bag recordings:

```
~/ros2_ws/
└── my_turtle_recording/       # Created by ros2 bag record
    ├── metadata.yaml
    └── my_turtle_recording_0.db3
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

> 💡 **Key Takeaway:** CLI tools only show you what's currently running. Always make sure you have nodes running before trying to inspect them!

**Next:** In Module 04, you'll write your first ROS2 nodes from scratch!