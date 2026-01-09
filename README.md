
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

- Ubuntu 24.04 (recommended, other versions may work)
- ROS2 Jazzy (other ROS2 distributions may work)
- Git
- Basic Linux terminal knowledge

Make sure ROS2 is properly installed and sourced:

```bash
source /opt/ros/jazzy/setup.bash
````

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

### Course Modules
```text
steps/
├── module_01_workspace.md   # Workspaces & Packages
├── module_02_nodes.md       # Nodes & Topics
├── module_03_services.md    # Services & Actions
├── module_04_commands.md    # ROS2 CLI Commands
├── module_05_urdf.md        # URDF Robot Description
├── module_06_xacro.md       # Xacro Macros
├── module_07_gazebo.md      # Gazebo Simulation
├── module_08_control.md     # ROS2 Control
├── module_09_sensors.md     # Camera Sensors
└── module_10_rviz.md        # RViz2 Visualization
```

### Reading Order (Important)

**Prerequisites** (complete before any module):

1. **introduction.md** - High-level overview of ROS2 distributions and useful links.
2. **installation.md** - Installation and environment setup.

⚠️ These two files **must be completed before starting any module**.

**Course Modules** (sequential order):

3. **module_01_workspace.md** - Course officially begins here with workspaces and packages.
4. **module_02_nodes.md → module_10_rviz.md** - Each module builds on the previous one.

This structure separates prerequisites from course content for clarity.


## Getting Started

The course is organized by **modules**, where each module lives in its **own Git branch**.

You will begin by cloning **Module 1** directly into your home directory.

From your terminal:

### Clone Module 1

You can clone a specific module branch using one of the following methods.

#### Option 1 – SSH (recommended)

```bash
cd ~
git clone --branch module_1 git@github.com:onlyshoky/ros2-tutorial.git
cd ros2-tutorial
```

#### Option 2 – GitHub CLI

```bash
cd ~
gh repo clone onlyshoky/ros2-tutorial
cd ros2-tutorial
git checkout module_1
```

#### Option 3 – HTTPS

```bash
cd ~
git clone --branch module_1 https://github.com/onlyshoky/ros2-tutorial.git
cd ros2-tutorial
```

At this point, you are ready to start **Module 1**.


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

Created by **Shoky**
GitHub: [https://github.com/onlyshoky](https://github.com/onlyshoky)

If you find this tutorial useful, feel free to star the repository ⭐

Happy hacking with ROS2 🚀


