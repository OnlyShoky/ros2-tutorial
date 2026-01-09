
This guide explains step-by-step how to install ROS 2 on Ubuntu. Each command is explained so you understand **what it does** and **why it's necessary**.

---

## 🪟 For Windows Users: Setting Up Ubuntu on WSL2

### **Why WSL2 for ROS 2?**
Windows Subsystem for Linux 2 (WSL2) allows you to run a Linux environment directly on Windows without dual-booting. This is perfect for ROS 2 development on Windows.

### **Step 1: Enable WSL2**
```bash
# Open PowerShell as Administrator and run:
wsl --install
```

```bash
# Check version:
wsl --version
```

### **Step 2: Install Ubuntu 24.04 (for Jazzy) or other versions**
```bash
# Check available distributions
wsl --list --online

# Install Ubuntu 24.04 (for ROS 2 Jazzy)
wsl --install -d Ubuntu-24.04

# Or install Ubuntu 22.04 (for ROS 2 Humble)
# wsl --install -d Ubuntu-22.04

# List installed distributions
wsl --list --verbose
```

### **Step 3: Set WSL2 as default version**
```bash
wsl --set-default-version 2
```

> ⚠️ **Important**: ROS 2 Jazzy requires **Ubuntu 24.04**. Choose your distribution based on which ROS 2 version you want to install.

### Opening Linux from Windows

**From the Start Menu**
Search for "Ubuntu 24.04" → click → Linux terminal opens.

**From PowerShell**
```powershell
wsl -d Ubuntu-24.04         # Opens Ubuntu
```

**From VS Code**
- Inside Ubuntu, run:
  ```bash
  code .
  ```
- Opens VS Code connected to WSL.
- Integrated terminal → Linux shell.
- Ideal for ROS and development.

**Open Linux folder in Windows Explorer**
- Direct path in Explorer: `\\wsl$\Ubuntu-24.04\home\your_username`
- From Ubuntu terminal: `explorer.exe .` opens the current folder in Explorer.

## 🚀 Quick Start

**Already have Ubuntu installed?** Jump directly to:
- [Locale Configuration](#1%EF%B8%8F⃣-locale-configuration) - If you're starting fresh
- [Install ROS 2](#6%EF%B8%8F⃣-install-ros-2-desktop) - If your system is already configured

---

## 1️⃣ Locale Configuration

### **Why do you need this?**
ROS 2 and many tools require **UTF-8** to properly handle special characters, text, and messages.

```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

> 🔍 **What `locale` does**: Displays all locale configuration variables.

---

## 2️⃣ Enable Universe Repository

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

> 🎯 **Important**: `software-properties-common` includes `add-apt-repository`.

---

## 3️⃣ Configure ROS Signing Key

### **Why do you need a GPG key?**
APT (Ubuntu's package manager) verifies that packages come from trusted sources using **cryptographic signatures**. The GPG key ensures ROS packages haven't been tampered with.

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

> 📍 **Standard location**: `/usr/share/keyrings/` is where Ubuntu stores GPG keys for official repositories.

---

## 4️⃣ Add ROS 2 Repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### **Command analysis:**
- `deb`: Indicates this is a binary package repository (not source code)
- `[arch=$(dpkg --print-architecture)]`: Automatically detects your architecture (amd64, arm64, etc.)
- `signed-by=...`: Specifies which GPG key to use for verification
- `http://packages.ros.org/ros2/ubuntu`: Official ROS 2 packages URL
- `$UBUNTU_CODENAME`: Automatically expands to "noble" for Ubuntu 24.04 (Jazzy)
- `main`: Main repository channel

> 💡 **Tip**: To see your architecture: `dpkg --print-architecture`.

---

## 5️⃣ Update System

```bash
sudo apt update
sudo apt upgrade
```

> ⚠️ **Note**: `upgrade` may take some time.

---

## 6️⃣ Install ROS 2 Desktop

**General Command:**
```bash
sudo apt install ros-{distro}-desktop
```

**For Our Tutorial (Jazzy):**
```bash
sudo apt install ros-jazzy-desktop
```

> 🕐 **Estimated time**: 10-30 minutes depending on your connection.

---

## 7️⃣ Configure Environment

**General Command:**
```bash
source /opt/ros/{distro}/setup.bash
```

**For Our Tutorial (Jazzy):**
```bash
source /opt/ros/jazzy/setup.bash
```

### **To make it permanent:**

**General Command:**
```bash
echo "source /opt/ros/{distro}/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**For Our Tutorial (Jazzy):**
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> 🔄 **Note**: Close and reopen your terminal for changes to take full effect.

---

## 8️⃣ Verify Installation

```bash
ros2
```

### **What should you see?**
A list of available subcommands:
- `run`: Run a node
- `topic`: Manage topics (messages)
- `node`: Manage nodes
- `bag`: Record/read data

> ✅ **Success!**: If you see this list, ROS 2 is installed and configured correctly.

---

## 🚨 Common Troubleshooting

### **Command `ros2` not found**
You forgot to run `source /opt/ros/{distro}/setup.bash` or didn't add it to your `.bashrc`.

### **Error: "Unable to locate package ros-{distro}-desktop"**
1. Verify you executed all steps in order
2. Ensure `sudo apt update` didn't produce errors
3. Confirm your Ubuntu version: `lsb_release -a`

### **Broken dependency issues**
```bash
# Try to fix dependencies
sudo apt --fix-broken install
sudo dpkg --configure -a
```

### **For Jazzy specific issues**
```bash
# Check Ubuntu version compatibility
lsb_release -a  # Must show Ubuntu 24.04 (noble)

# If on wrong Ubuntu version, consider:
# 1. Installing Ubuntu 24.04 via WSL2 (Windows)
# 2. Using a VM with Ubuntu 24.04
# 3. Dual-booting with Ubuntu 24.04
```

---

## 📦 ROS 2 Distribution Compatibility Matrix

| ROS 2 Distribution | Ubuntu Version | Release Date | Support Until |
|-------------------|----------------|--------------|---------------|
| **Jazzy**         | 24.04 (Noble)  | May 2024     | May 2029      |
| **Humble**        | 22.04 (Jammy)  | May 2022     | May 2027      |
| **Iron**          | 22.04 (Jammy)  | May 2023     | Nov 2024      |
| **Rolling**       | Rolling        | Continuous   | Continuous    |
