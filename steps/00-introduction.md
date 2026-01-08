
This document helps you choose the appropriate ROS 2 distribution
depending on your operating system, project goals, and long-term support needs.

---

## What is a ROS 2 Distribution?

A ROS 2 distribution is a **versioned set of ROS 2 packages** released together.

It is similar to a Linux distribution:
- It provides a stable and tested software baseline
- Core packages receive bug fixes and non-breaking updates
- APIs remain stable during the support period

Once a distribution is released, major changes are avoided to ensure
long-term maintainability.

---

## Recommended Operating System

ROS 2 is primarily developed and tested on **Ubuntu Linux**.

Recommended Ubuntu versions:

| Ubuntu Version | Support Status |
|---------------|----------------|
| Ubuntu 24.04 (Noble) | ✅ Recommended |
| Ubuntu 22.04 (Jammy) | ✅ Supported |
| Ubuntu 20.04 (Focal) | ⚠️ Legacy |

For beginners, **Ubuntu 22.04 or 24.04** is strongly recommended.

---

## Current ROS 2 Distributions

| Distribution         | Release Date | End of Life | Ubuntu | Notes                          |
| -------------------- | ------------ | ----------- | ------ | ------------------------------ |
| **Jazzy Jalisco**    | May 2024     | May 2029    | 24.04  | Long-term support, recommended |
| **Humble Hawksbill** | May 2022     | May 2027    | 22.04  | Stable LTS, widely used        |
| Iron Irwini          | May 2023     | Dec 2024    | 22.04  | Short-term support             |
| Galactic Geochelone  | May 2021     | Dec 2022    | 20.04  | End of life                    |
| Foxy Fitzroy         | June 2020    | June 2023   | 20.04  | End of life                    |

---

## Which Distribution Should I Choose?

### Beginners
**Jazzy Jalisco (Ubuntu 24.04)**  
or  
**Humble Hawksbill (Ubuntu 22.04)**

These offer:
- Long-term support
- Stable APIs
- Extensive documentation
- Large community adoption

### Learning & Tutorials
Humble or Jazzy are recommended, as most learning resources target them.

### Production & Industry
Choose an **LTS distribution** (Humble or Jazzy) to ensure:
- Long maintenance window
- API stability
- Security updates

---

## Key Features Across ROS 2 Distributions

All modern ROS 2 distributions provide:

- Node-based architecture
- Publish / subscribe communication
- Services and actions
- DDS-based middleware
- Real-time support
- Multi-robot communication
- Lifecycle-managed nodes
- Cross-platform build system (colcon)

Newer distributions improve performance, tooling, and platform support.

---

# Summary

## Changes Carried Over from Iron into Jazzy

Jazzy consolidates many improvements first introduced in Iron:

- Faster middleware discovery
- Improved DDS interoperability
- Better logging and diagnostics
- More robust executor internals

Iron was short-term; Jazzy makes these improvements **production-ready**.

---

## What Humble Introduced (Context)

Humble was a major stabilization release.

Key additions at the time:
- Mature lifecycle nodes
- Improved launch and launch_ros
- Content-filtered topics
- Security improvements (SROS2)
- Broad industry adoption

Humble remains **stable and valid**, but lacks later refinements.

---

## Practical Recommendation

### Use Jazzy if:
- You are starting a new ROS 2 project
- You are learning ROS 2 today
- You want the longest support window
- You are targeting modern hardware and tooling

### Use Humble if:
- You are maintaining an existing system
- You rely on Ubuntu 22.04-only drivers
- Migration cost is currently too high

---

## Distribution Used in This Tutorial Series

This tutorial series uses **ROS 2 Jazzy Jalisco**.

Reasons:
- Latest LTS
- Improved core libraries
- Better developer experience
- Longer long-term support

All examples are designed to be **forward-compatible**.


## **Useful Resources & References**

#### **WSL2 Documentation**
- [Microsoft WSL Documentation](https://docs.microsoft.com/en-us/windows/wsl/) - Official WSL docs
- [WSL2 Installation Guide](https://docs.microsoft.com/en-us/windows/wsl/install) - Step-by-step installation
- [WSLg Documentation](https://github.com/microsoft/wslg) - Linux GUI apps on WSL2

#### **Ubuntu & Linux Resources**
- [Ubuntu Documentation](https://help.ubuntu.com/) - Official Ubuntu guides
- [Ubuntu 24.04 Release Notes](https://discourse.ubuntu.com/t/noble-numbat-release-notes/39834) - What's new in Ubuntu 24.04

#### **ROS 2 Resources**
- [ROS 2 Official Documentation](https://docs.ros.org/) - Complete ROS 2 documentation
- [ROS 2 Releases](https://docs.ros.org/en/rolling/Releases.html) - Distribution information and support timelines
- [ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html) - Installation instructions for all distributions

#### **Development Tools**
- [VS Code WSL Extension](https://code.visualstudio.com/docs/remote/wsl) - Develop in WSL from VS Code
- [Windows Terminal](https://docs.microsoft.com/en-us/windows/terminal/) - Modern terminal for Windows (great with WSL)
- [Docker Desktop with WSL2](https://docs.docker.com/desktop/windows/wsl/) - Run Docker containers with WSL2 backend

#### **Troubleshooting**
- [WSL2 Common Issues](https://docs.microsoft.com/en-us/windows/wsl/troubleshooting) - Troubleshooting guide
- [WSL2 GitHub Issues](https://github.com/microsoft/WSL/issues) - Report bugs and issues
- [ROS 2 Troubleshooting](https://docs.ros.org/en/humble/How-To-Guides.html) - Common ROS 2 problems and solutions

#### **Community & Learning**
- [ROS Discourse](https://discourse.ros.org/) - ROS community forum
- [ROS Answers](https://answers.ros.org/) - Q&A for ROS
- [r/ROS on Reddit](https://www.reddit.com/r/ROS/) - ROS subreddit
- [r/bashonubuntuonwindows](https://www.reddit.com/r/bashonubuntuonwindows/) - WSL community
