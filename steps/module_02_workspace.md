# Module 02: Setting Up Your Workspace 🏗️

Now that you've seen ROS2 in action with TurtleSim, let's set up your development environment. Think of this as building your workshop before you start creating.

---

## 🎯 What You Will Learn

- Understand the ROS 2 workspace structure
- Create and configure a workspace
- Build packages with `colcon`
- Create your own ROS 2 package
- Understand `package.xml` and `CMakeLists.txt`

---

## 📦 What is a ROS 2 Workspace?

A **workspace** is a directory where you develop, build, and run your ROS 2 code. Think of it as your project folder that contains all your custom packages.

### Workspace Structure

```
~/ros2_ws/                  # Your workspace root
├── src/                    # Source code (your packages go here)
│   ├── package_1/
│   └── package_2/
├── build/                  # Build artifacts (auto-generated)
├── install/                # Installed packages (auto-generated)
└── log/                    # Build logs (auto-generated)
```

> 💡 **Tip:** You only edit files in `src/`. The other folders are generated automatically when you build.

---

## 1️⃣ Prerequisites

Before creating a workspace, ensure you have ROS 2 Jazzy installed and sourced:

```bash
# Verify installation
ros2
```

Install the colcon build tool and other development dependencies:

```bash
# Install colcon and build tools
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep -y
```

> 📍 **What is colcon?** It's the build tool for ROS 2, similar to how `make` or `cmake` work for C++ projects. It handles compiling all your packages in the correct order.

---

## 2️⃣ Create Your Workspace

Let's create a workspace called `ros2_ws`:

```bash
# Create the workspace directory structure
mkdir -p ~/ros2_ws/src

# Navigate to the workspace
cd ~/ros2_ws
```

### Initialize rosdep (First Time Only)

`rosdep` helps you install system dependencies required by ROS 2 packages:

```bash
# Initialize rosdep (run only once after installing ROS 2)
sudo rosdep init

# Update rosdep database
rosdep update
```

> ⚠️ **Note:** If you get an error that rosdep is already initialized, you can skip `sudo rosdep init` and just run `rosdep update`.

---

## 3️⃣ What is a ROS 2 Package?

A **package** is the organizational unit for your ROS 2 code. Each package contains:

| Component | Purpose |
|-----------|---------|
| `package.xml` | Package metadata and dependencies |
| `CMakeLists.txt` | Build configuration (for C++ packages) |
| `setup.py` | Build configuration (for Python packages) |
| `src/` or `scripts/` | Your source code |

### Package Types

| Type | Build System | Use Case |
|------|--------------|----------|
| `ament_cmake` | CMake | C++ packages |
| `ament_python` | setuptools | Python packages |

---

## 4️⃣ Create Your First Package

Let's create a package for our robot arm project. Navigate to the `src` folder:

```bash
cd ~/ros2_ws/src
```

### Create a C++ Package

```bash
# Create a C++ package with dependencies
ros2 pkg create simple_arm_description \
  --build-type ament_cmake \
  --dependencies rclcpp std_msgs \
  --description "Simple robot arm for ROS 2 Jazzy tutorial"
```

### Create a Python Package

```bash
# Create a Python package
ros2 pkg create simple_arm_control \
  --build-type ament_python \
  --dependencies rclpy std_msgs \
  --description "Control nodes for simple robot arm"
```

> 🔍 **Command breakdown:**
> - `ros2 pkg create`: Creates a new package
> - `--build-type`: Specifies C++ (`ament_cmake`) or Python (`ament_python`)
> - `--dependencies`: Packages your code depends on
> - `--description`: Brief package description

---

## 5️⃣ Understanding Package Files

### Package Structure (C++)

```
simple_arm_description/
├── CMakeLists.txt          # Build instructions
├── package.xml             # Metadata and dependencies
├── include/                # Header files (.hpp)
│   └── simple_arm_description/
├── src/                    # Source files (.cpp)
└── launch/                 # Launch files (create manually)
```

### Package Structure (Python)

```
simple_arm_control/
├── setup.py                # Build and install configuration
├── setup.cfg               # Package configuration
├── package.xml             # Metadata and dependencies
├── resource/               # Package marker
│   └── simple_arm_control
├── simple_arm_control/     # Python modules
│   └── __init__.py
└── test/                   # Test files
```

---

## 6️⃣ Configuring package.xml

The `package.xml` file defines your package's identity and dependencies. Here's an example for the robot arm package:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>simple_arm_description</name>
  <version>0.0.1</version>
  <description>Simple robot arm description for ROS 2 Jazzy tutorial</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>urdf</depend>
  <depend>xacro</depend>
  <depend>robot_state_publisher</depend>

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Dependency Types

| Tag | When Installed | Use Case |
|-----|----------------|----------|
| `<buildtool_depend>` | At build time | Build tools (ament_cmake) |
| `<build_depend>` | At build time | Compile-time dependencies |
| `<exec_depend>` | At runtime | Runtime dependencies |
| `<depend>` | Both | Build and runtime (most common) |
| `<test_depend>` | For testing | Test frameworks |

---

## 7️⃣ Configuring CMakeLists.txt (C++ Packages)

For C++ packages, `CMakeLists.txt` defines how to build your code:

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_arm_description)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories (URDF, launch files, etc.)
install(
  DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

---

## 8️⃣ Building the Workspace

### Build All Packages

From the workspace root directory:

```bash
cd ~/ros2_ws

# Build all packages
colcon build
```

### Build Specific Packages

To save time, build only the packages you're working on:

```bash
# Build only the arm description package
colcon build --packages-select simple_arm_description
```

### Useful Build Options

```bash
# Build with symbolic links (faster for Python development)
colcon build --symlink-install

# Continue building even if some packages fail
colcon build --continue-on-error

# See detailed output
colcon build --event-handlers console_direct+
```

> 🕐 **Expected output:** You should see `Summary: X packages finished` when the build completes successfully.

---

## 9️⃣ Sourcing Your Workspace

After building, you must **source** the workspace to use your packages:

```bash
# Source the workspace overlay
source ~/ros2_ws/install/setup.bash
```

### Make It Permanent

Add the source command to your `.bashrc` so it runs automatically:

```bash
# Add to .bashrc (run once)
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Reload .bashrc
source ~/.bashrc
```

> ⚠️ **Important:** Always source ROS 2 **before** your workspace:
> ```bash
> source /opt/ros/jazzy/setup.bash      # ROS 2 base
> source ~/ros2_ws/install/setup.bash   # Your workspace (overlay)
> ```

---

## 🔟 Verify Your Setup

Check that your packages are recognized:

```bash
# List all available packages (should include yours)
ros2 pkg list | grep simple_arm

# Get package information
ros2 pkg prefix simple_arm_description
```

Expected output:
```
/home/your_user/ros2_ws/install/simple_arm_description
```

---

## 🚨 Common Troubleshooting

### Package Not Found

**Problem:** `ros2 pkg list` doesn't show your package
**Solution:**
```bash
# Rebuild and source again
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Build Fails with Missing Dependencies

**Problem:** Build fails because of missing system packages
**Solution:**
```bash
# Install dependencies automatically
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro jazzy -y
```

### Conflicting Package Versions

**Problem:** Strange behavior or version conflicts
**Solution:**
```bash
# Clean and rebuild from scratch
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

---

## 📁 Module File Structure

After completing this module, your workspace should look like:

```
~/ros2_ws/
├── src/
│   ├── simple_arm_description/     # C++ package
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── include/
│   │   │   └── simple_arm_description/
│   │   └── src/
│   └── simple_arm_control/         # Python package
│       ├── setup.py
│       ├── setup.cfg
│       ├── package.xml
│       ├── resource/
│       │   └── simple_arm_control
│       ├── simple_arm_control/
│       │   └── __init__.py
│       └── test/
├── build/                          # (auto-generated)
├── install/                        # (auto-generated)
└── log/                            # (auto-generated)
```

---

## ✅ Summary

In this module, you learned:

- **Workspaces** organize your ROS 2 development projects
- **Packages** are the building blocks containing your code
- Use `ros2 pkg create` to generate package templates
- `colcon build` compiles your packages
- Always **source** your workspace after building
- `rosdep` helps install system dependencies

**Next:** In Module 03, we'll explore how to inspect the ROS2 system with CLI tools (remember those commands from the TurtleSim demo?).
