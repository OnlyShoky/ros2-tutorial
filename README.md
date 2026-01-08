
# ROS2 Modular Tutorial

Welcome to this **ROS2 Modular Tutorial**.  
This repository is designed to help you learn **ROS2 step by step**, following a **modular and practical approach**.

The goal of this project is that you can:
- Understand core ROS2 concepts
- Build real ROS2 packages
- Follow a clear learning path using written modules
- Work inside a clean and isolated ROS2 workspace

This tutorial is suitable for **beginners to intermediate users** who want a structured way to learn ROS2.

***

## Prerequisites

Before starting, make sure you have:

- Ubuntu 24.04 (recommended) or others
- ROS2 (Jazzy) or others
- Git
- Basic Linux terminal knowledge

Make sure ROS2 is properly installed and sourced:

```bash
source /opt/ros/humble/setup.bash
````

---

---

## Repository Structure

```text
ros2-tutorial/
├── README.md
├── steps/        # Step-by-step tutorial (Markdown)
├── ros2_ws/      # ROS2 workspace used in this tutorial
│   └── src/      # ROS2 packages live here
└── cheatsheet/   # Optional cheatsheets that you can use.
```

* `steps/` contains all the written modules (`.md`)
* `ros2_ws/` is a **real ROS2 workspace**
* Each module builds on top of the previous one

---

## Getting Started

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

---

## Following the Tutorial

All learning content is located inside the `steps/` directory.

All videos can be found in my (ROS2 Tutorial course)[https://www.youtube.com/playlist?] playlist.

Start here:

```text
steps/00-introduccion.md
```

Each step:

* Explains a single concept
* Includes commands you can copy/paste
* Refers directly to the code inside `ros2_ws/src`

Follow the steps **in order**.

---

## Important Notes

* This workspace is **only for this tutorial**
* Do not mix it with other ROS2 workspaces
* Always source the correct setup files
* If something breaks, you can safely delete the repository and clone it again

---

## Author

Created by **Shoky**
GitHub: [https://github.com/onlyshoky](https://github.com/onlyshoky)

If you find this tutorial useful, feel free to star the repository ⭐

Happy hacking with ROS2 🚀

