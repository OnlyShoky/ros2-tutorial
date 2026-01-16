# Module 01: Your First ROS2 Experience 🐢

Welcome to ROS2! In this module, you'll see ROS2 in action before diving into any theory. Let's run a classic demo that shows the power of ROS2 communication.

---

## 🎯 What You Will Learn

- Run your first ROS2 application
- Control a simulated robot with keyboard
- See ROS2 nodes communicating in real-time
- Get excited about what you'll build!

---

## 🐢 Meet TurtleSim

**TurtleSim** is a simple 2D robot simulator that comes with ROS2. It's perfect for learning because:

- Immediate visual feedback
- Shows real ROS2 concepts (nodes, topics, services)
- No complex setup required

---

## 1️⃣ Verify TurtleSim installation

```bash
ros2 pkg executables turtlesim
```

Expected output:
```
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node
```

---

## 2️⃣ Run the Simulation

Open a terminal and start the turtlesim window:

```bash
ros2 run turtlesim turtlesim_node
```

🎉 **You should see a window with a turtle in the center!**

> 💡 **What just happened?** You launched a ROS2 **node** called `turtlesim_node`. This node creates the simulation window and controls the turtle.

---

## 3️⃣ Control the Turtle

Open a **second terminal** (keep the first one running!) and start the keyboard controller:

```bash
ros2 run turtlesim turtle_teleop_key
```

Now use the arrow keys to move the turtle around!

| Key | Action |
|-----|--------|
| ↑ | Move forward |
| ↓ | Move backward |
| ← | Turn left |
| → | Turn right |

> 🔍 **What's happening behind the scenes?** The `turtle_teleop_key` node is sending **messages** to the `turtlesim_node` through a **topic**. This is how ROS2 robots communicate!

---

## 4️⃣ See What's Running

Open a **third terminal** and peek at the ROS2 system:

```bash
# List all running nodes
ros2 node list
```

Output:
```
/teleop_turtle
/turtlesim
```

Two nodes are running - exactly what we started!

```bash
# List all topics (communication channels)
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

> 💡 The `/turtle1/cmd_vel` topic carries the movement commands from your keyboard to the turtle!

---

## 5️⃣ Spawn Another Turtle

ROS2 also has **services** for one-time requests. Let's spawn a second turtle:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```

🎉 **A second turtle appears!**

---

## 6️⃣ Watch the Data Flow

See the velocity commands in real-time:

```bash
ros2 topic echo /turtle1/cmd_vel
```

Now move the turtle with your arrow keys and watch the terminal - you'll see the actual data being sent!

Press `Ctrl+C` to stop.

---

## 🎯 What You Just Experienced

| Concept | Example |
|---------|---------|
| **Node** | `turtlesim_node`, `teleop_turtle` |
| **Topic** | `/turtle1/cmd_vel` (continuous data) |
| **Service** | `/spawn` (one-time request) |
| **Message** | Velocity commands sent via topic |

These are the core concepts of ROS2. You'll understand them deeply in later modules!

---

## 🚨 Troubleshooting

### Window Doesn't Appear

**Problem:** Turtlesim runs but no window shows
**Solution (WSL2 users):**
```bash
# Check WSLg is working
echo $DISPLAY

# If empty, try:
export DISPLAY=:0
```

### Command Not Found

**Problem:** `ros2: command not found`
**Solution:**
```bash
source /opt/ros/jazzy/setup.bash
```

---

## 📁 Module File Structure

This module doesn't create any files - it's all about experiencing ROS2!

```
(No new files created)
```

---

## ✅ Summary

In this module, you:

- Ran your **first ROS2 application** (turtlesim)
- Controlled a robot with **keyboard teleop**
- Saw **nodes** communicating via **topics**
- Used a **service** to spawn a new turtle
- Peeked at the ROS2 system with CLI tools

**Next:** In Module 02, we'll set up your own ROS2 workspace so you can start building your own projects!
