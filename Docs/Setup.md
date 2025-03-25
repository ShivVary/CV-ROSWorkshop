# 🛠️ Setup Guide

This guide walks you through setting up your environment to run ROS 2 for the workshop.  
You can choose between three options depending on your system and preferences.

---

## 🖥️ Option 1: Use a Pre-Configured VM (HIGHLY Recommended)

If you want to skip installation and dive straight into ROS 2, download the pre-configured virtual machine image.

### 🔗 Downloads

- 📦 **VirtualBox** (hypervisor): [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
- 🧳 **Pre-configured VM image** (Google Drive): [Download VM Image](https://drive.google.com/your-placeholder-link)

> 💡 ** Make sure virtualization is enabled in your BIOS/firmware.**
- If you do not know how to do this contact a volunteer

### 🚀 Steps

1. Install VirtualBox
2. Download the `.ova` file from the link above
3. Open VirtualBox and go to **File → Import Appliance**
4. Select the downloaded `.ova` file and follow the prompts
5. Launch the VM and you're ready to go!

> 📝 Your VM already includes ROS 2 Humble, essential tools, and common dependencies.

---

## 🍎 Option 2: Use Ubuntu Environment on macOS

If you're using a Mac, you can run Ubuntu in a virtual machine.

### 🔗 Tools

- 💻 **VirtualBox**: [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
- 🟣 **UTM for Apple Silicon (M1/M2)**: [Download UTM](https://mac.getutm.app/)
- 📥 **Ubuntu 22.04 ISO (Google Drive)**: [Download ISO](https://drive.google.com/your-ubuntu-iso-link)

### 🧭 Instructions

1. Install VirtualBox or UTM
2. Download the Ubuntu 22.04 ISO from the link above
3. Create a new VM in your tool of choice
4. Mount the ISO file as the boot disk
5. Follow the on-screen instructions to install Ubuntu

> Once Ubuntu is installed, continue with [ROS 2 installation steps](#option-3-install-ubuntu-manually).

---

## 💽 Option 3: Install Ubuntu 22.04 Manually

Want to install Ubuntu 22.04 on bare metal or another VM setup?

### 🔗 Resources

- 🌐 [Ubuntu 22.04 Official Download Page](https://ubuntu.com/download/desktop)
- 📥 [Google Drive ISO Backup (optional)](https://drive.google.com/your-ubuntu-iso-link)
---

### 🔧 Installation Steps

Once you’ve installed Ubuntu, copy these instruction to your terminal to install ROS 2 Humble, Gazebo, essential tools, and common dependencies:

```bash
 cd ~
 git clone https://github.com/ShivVary/CV-ROSWorkshop.git
 cd ~/CV-ROSWorkshop/"Installation Files"
 chmod +x install_combo.sh
 ./install_combo.sh
 source ~/.bashrc
 ```
 ### Test Run in the VM
 ```
 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
 ```