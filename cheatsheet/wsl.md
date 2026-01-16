

### **1️⃣ What is WSL2?**
WSL2 (Windows Subsystem for Linux 2) is a feature of Windows that allows you to run a real Linux kernel inside Windows 10/11, without the need for a full virtual machine.

- Runs full Linux distributions like Ubuntu.
- Enables the use of `apt`, `bash`, ROS 2, Python, etc.
- Runs within Windows but with Linux file system and kernel.
- Supports Linux GUI apps thanks to WSLg.

**WSL2 vs. Hyper-V**

| Feature          | WSL2                                                                 | Hyper-V                                                      |
|------------------|----------------------------------------------------------------------|--------------------------------------------------------------|
| Virtualization   | Lightweight, integrated with Windows                                 | Full virtualization, requires dedicated VM                   |
| Kernel           | Real Linux kernel inside Windows                                     | Linux/Windows running inside VM                             |
| Performance      | Fast, direct disk access                                             | Heavier, dedicated CPU/RAM                                  |
| GUI Support      | Yes, with WSLg                                                       | Only if VM has a desktop environment                         |
| Typical Use      | Development, ROS, Linux tools                                        | Complete isolated machines, testing                          |

---

### **2️⃣ Installing WSL2 and Ubuntu 24.04**
**Commands in PowerShell (as Administrator)**

```powershell
# Install WSL2 and the latest Ubuntu
wsl --install -d Ubuntu-24.04

# List installed distributions
wsl --list --verbose

# Terminate a running distribution
wsl --terminate Ubuntu-24.04

# Shut down all WSL2 distros
wsl --shutdown

# Completely remove Ubuntu
wsl --unregister Ubuntu-24.04
```

**First boot of Ubuntu**
When you open Ubuntu for the first time, you'll be asked to create a Linux username and password.

---

### **3️⃣ Checking Your Ubuntu and Linux Environment**
```bash
# Check Ubuntu version
lsb_release -a

# Kernel info
uname -a

# Detailed system info (requires installing neofetch)
sudo apt install neofetch -y
neofetch

# Kernel version info
cat /proc/version
```

**Try some Linux-only tools**
```bash
# Linux process monitor
sudo apt install htop -y
htop  # Exit with F10

# Simple Linux GUI app
sudo apt install x11-apps -y
xeyes
```

---

### **4️⃣ File Structure in WSL2**
**Access Windows files from Linux**
- Your Windows drives are mounted under `/mnt/`
- Example: `C:` drive is at `/mnt/c/`
- Access all your Windows files from Linux:  
  `ls /mnt/c/Users/your_username`

**Linux's own files**
- Your Linux home directory: `/home/your_linux_username`
- Recommended for projects (ROS, Linux development, etc.)
- Initially empty — use `ls -la` to see hidden files (starting with `.`)

---

### **5️⃣ Opening Linux from Windows**

**5.1 From the Start Menu**
Search for "Ubuntu 24.04" → click → Linux terminal opens.

**5.2 From PowerShell**
```powershell
wsl -d Ubuntu-24.04         # Opens Ubuntu
wsl -d Ubuntu-24.04 ls      # Runs a command and exits
```

**5.3 From VS Code**
- Inside Ubuntu, run:
  ```bash
  code .
  ```
- Opens VS Code connected to WSL.
- Integrated terminal → Linux shell.
- Ideal for ROS and development.

**5.4 Open Linux folder in Windows Explorer**
- Direct path in Explorer: `\\wsl$\Ubuntu-24.04\home\your_username`
- From Ubuntu terminal: `explorer.exe .` opens the current folder in Explorer.