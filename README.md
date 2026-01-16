
# ROS2 Tutorial

Welcome to this **ROS2 Tutorial**.  
This repository is designed to help you learn **ROS2 step by step**, following a **modular and practical approach**.

The goal of this project is to help you:

- Understand core ROS2 concepts
- Build real ROS2 packages
- Follow a clear learning path using written modules
- Work inside a clean and isolated ROS2 workspace

This tutorial is suitable for **beginners to intermediate users** who want a structured way to learn ROS2.

## Important: Workspace Setup

To maintain consistency throughout the tutorial, we recommend cloning this repository directly as `ros2_ws` in your home directory:

```bash
# Navigate to your home directory
cd ~

# Clone and rename in one step
git clone https://github.com/onlyshoky/ros2-tutorial.git ros2_ws

# Navigate to your workspace
cd ros2_ws
```


## Prerequisites

Before starting, make sure you have:

### **For Linux Users:**
- Ubuntu 24.04 (recommended)  
  *Note: Other distributions might not be compatible with ROS2 Jazzy. Check `introduction.md` for distribution compatibility.*

### **For Windows Users:**
- Windows 10 or 11 with WSL2 installed
- Follow the WSL2 setup guide in `installation.md`

### **Common Requirements:**
- Git installed

If you haven't installed ROS2 yet, follow the instructions in `steps/installation.md`.



## Repository Structure

After cloning as `ros2_ws`, your structure will be:

```
ros2_ws/
├── README.md
├── steps/        # Step-by-step written tutorial (Markdown)
├── src/          # ROS2 packages live here (initially empty)
└── cheatsheet/   # Optional cheatsheets you can use
```

* `steps/` contains all the written guides and modules
* `ros2_ws/` is a **real ROS2 workspace**
* Each module builds on top of the previous one


## How to Use the Steps Directory

All the written content for this course is located inside the `steps/` directory.

The files are intentionally ordered to guide you through the course in the correct sequence:

### Preliminary Files (Prerequisites)
```text
steps/
├── introduction.md      # ROS2 overview and distributions
└── installation.md      # Environment setup guide
```

### Course Modules (13 Total)
```
steps/
├── module_01_turtlesim.md   # 🐢 First ROS2 Experience
├── module_02_workspace.md   # 🏗️ Workspace Setup
├── module_03_cli.md         # 🔍 CLI Exploration
├── module_04_first_node.md  # ✍️ Your First Node
├── module_05_topics.md      # 📡 Topics Deep Dive
├── module_06_services.md    # 🔄 Services & Actions
├── module_07_launch.md      # 🚀 Launch Files
├── module_08_urdf.md        # 🤖 URDF Robot Description
├── module_09_xacro.md       # 🧩 Xacro Macros
├── module_10_gazebo.md      # 🌍 Gazebo Simulation
├── module_11_control.md     # ⚙️ ROS2 Control
├── module_12_sensors.md     # 📷 Sensors & Camera
└── module_13_rviz.md        # 👁️ RViz2 Visualization
```

### Reading Order (Important)

**Prerequisites** (complete before any module):
1. **introduction.md** - High-level overview of ROS2
2. **installation.md** - Environment setup

⚠️ These two files **must be completed before starting any module**.

**Course Modules** (sequential order):
Start with **module_01_turtlesim.md** and progress through modules in order. Each builds on the previous one.


## Getting Started

The course is organized by **modules**, where each module lives in its **own Git branch**.

You will begin by cloning **Module 1** directly into your home directory.

From your terminal:

## Getting Started

### If Following the Written Tutorial:
```bash
# Clone the repository
cd ~
git clone https://github.com/onlyshoky/ros2-tutorial.git ros2_ws
cd ros2_ws

# Start reading the steps
cat steps/introduction.md
```

### If Cloning a Specific Module:
```bash
# Clone Module 1 with complete code
cd ~
git clone --branch module_1 https://github.com/onlyshoky/ros2-tutorial.git ros2_ws
cd ros2_ws
```


## Building the ROS2 Workspace

Each module contains a ROS2 workspace that must be built.

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

If the build finishes without errors, you can continue with the tutorial.


## Following the Tutorial

All learning content is located inside the `steps/` directory.

All videos can be found in my **ROS2 Tutorial** YouTube playlist:
[YouTube Playlist](https://www.youtube.com/playlist?list=PLN65mHMMQSYDTAgwF_k2IJLSSkxMcROEL)

Start here:

```text
steps/introduction.md
```

Each step:

* Explains a single concept
* Includes commands you can copy and paste

Follow the steps **in order**.



## Moving to the Next Module

When you are ready to continue to the next module, simply switch branches:

```bash
git checkout module_2
```

Then rebuild the workspace if needed:

```bash
cd ros2_ws
colcon build
source install/setup.bash
```

Repeat this process for each module.


## Important Notes

* This workspace is **only for this tutorial**
* Do not mix it with other ROS2 workspaces
* Always source the correct ROS2 and workspace setup files
* If something breaks, you can safely delete the repository and clone the module again


## Author

Created by **Mohamed EL MOURABIT**
GitHub: [https://github.com/onlyshoky](https://github.com/onlyshoky)

If you find this tutorial useful, feel free to star the repository ⭐

Happy hacking with ROS2 🚀


