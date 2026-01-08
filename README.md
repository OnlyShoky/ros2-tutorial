
# ROS2 Tutorial

Welcome to this **ROS2 Tutorial**.  
This repository is designed to help you learn **ROS2 step by step**, following a **modular and practical approach**.

The goal of this project is to help you:

- Understand core ROS2 concepts
- Build real ROS2 packages
- Follow a clear learning path using written modules
- Work inside a clean and isolated ROS2 workspace

This tutorial is suitable for **beginners to intermediate users** who want a structured way to learn ROS2.


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

```text
ros2-tutorial/
├── README.md
├── steps/        # Step-by-step written tutorial (Markdown)
├── ros2_ws/      # ROS2 workspace used in this tutorial
│   └── src/      # ROS2 packages live here
└── cheatsheet/   # Optional cheatsheets you can use
```

* `steps/` contains all the written guides and modules
* `ros2_ws/` is a **real ROS2 workspace**
* Each module builds on top of the previous one


## How to Use the Steps Directory

All the written content for this course is located inside the `steps/` directory.

The files are intentionally ordered to guide you through the course in the correct sequence:

```text
steps/
├── 00-introduction.md
├── 01-installation.md
│
├── module_1.md
├── module_2.md
├── module_3.md
```

### Reading Order (Important)

1. **00-introduction.md**
   Provides a high-level overview of the ROS2 distributions and some useful links.

2. **01-installation.md**
   Guides you through the installation and environment setup required before starting the course.

⚠️ These two files **must be completed before starting any module**.

3. **module_1.md**
   This is where the course officially begins. It introduces the first ROS2 concepts and practical examples.

4. **module_2.md**, **module_3.md**, …
   Each module builds on top of the previous one and introduces new ROS2 concepts and packages.

This structure makes it clear what must be read **before** starting the modules and helps avoid common setup issues.


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
[https://www.youtube.com/playlist?list=YOUR_PLAYLIST_HERE](https://www.youtube.com/playlist?list=YOUR_PLAYLIST_HERE)

Start here:

```text
steps/00-introduction.md
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


