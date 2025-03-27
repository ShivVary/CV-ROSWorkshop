# 🛠️ Setup Guide

This guide walks you through setting up your environment to run ROS 2 for the workshop.  
You can choose between three options depending on your system and preferences.

---

## 🖥️ Option 1: Use a Pre-Configured VM (HIGHLY Recommended)

If you want to skip installation and dive straight into ROS 2, download the pre-configured virtual machine image.

### 🔗 Downloads

- 📦 **VirtualBox** (hypervisor): [Download VirtualBox](https://www.virtualbox.org/wiki/Downloads)
- 🧳 **Pre-configured VM image** (Google Drive): [Download VM Image](https://drive.google.com/file/d/1PXVDw4N5lxZUJlv4mMCjEZXLyQYma61p/view?usp=sharing)
- 📥 [**Ubuntu 22.04 ISO**](https://drive.google.com/file/d/1Eje4dw9OXfiPicqhlAbsmVYbvwIku_G1/view?usp=sharing)

> 💡 Make sure virtualization is enabled in your BIOS/firmware.
> If you do not know how to do this contact a volunteer

### 🚀 Steps

1. Install VirtualBox
2. Download the `.ova` file from the link above
> 💡 Make sure you have 60 gb of free storage

3. Open VirtualBox and go to **File → Import Appliance**
4. Select the downloaded `.ova` file and follow the prompts

** You might want to update the the storage, cpu (cores), ram required. As of now it is set to utilize 4 cores, 4 gb ram and 50 gb storage). If you do not know how to do this contact a volunteer**

5. Launch the VM and you're ready to go!

> 📝 Your VM already includes ROS 2 Humble, essential tools, and common dependencies.

6. Update and upgrade dependencies

```bash
sudo apt update && sudo apt upgrade -y
```

7. Source ROS2 setup.bash

``` bash
Source ~/.bashrc
```

8. Test Run in the VM
 
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 🍎 Option 2: Use Ubuntu Environment on macOS

If you're using a Mac (why), you can run Ubuntu in a virtual machine.

### 🔗 Tools

- 🟣 **UTM for Apple Silicon (M1/M2)**: [Download UTM](https://mac.getutm.app/)

### 🧭 Instructions

1. Install UTM
2. Select the Ubuntu 22.04 ISO from the UTM store.
3. Create a new VM in your tool of choice
4. Mount the ISO file as the boot disk
5. Follow the on-screen instructions to install Ubuntu

> 📝 Your VM needs ROS 2 Humble, essential tools, and common dependencies.

6. 

```bash
 cd ~
 git clone https://github.com/ShivVary/CV-ROSWorkshop.git
 cd ~/CV-ROSWorkshop/"Installation Files"
 chmod +x install_combo.sh
 ./install_combo.sh
 source ~/.bashrc
 ```

7. Update and upgrade dependencies

```bash
sudo apt update && sudo apt upgrade -y
```

8. Source ROS2 setup.bash

``` bash
Source ~/.bashrc
```

9. Test Run in the VM
 
```bash
ros2 --version
```

---

## 💽 Option 3: Install Ubuntu 22.04 Manually (only Windows users)

Want to install Ubuntu 22.04 on bare metal or another VM setup?
> 📝 Takes time but rewarding

### 🔗 Resources

- 🌐 [Installing Ubuntu 22.04 instructions](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
- 📥 [Google Drive ISO Backup (optional)](https://drive.google.com/file/d/1Eje4dw9OXfiPicqhlAbsmVYbvwIku_G1/view?usp=sharing)

> 💡  Bootable USB drives with Ubuntu 22.04 is available during the workshop. If you’d like to install Ubuntu directly on your machine, ask a volunteer to assist you.

**Disk partition is risky can cause data wipe out. Do it at your own risk** 
> I have learnt it the hard way. 

---

### 🔧 Installation Steps

Once you’ve sucessfully installed Ubuntu 22.04, copy these instruction to your terminal to install ROS 2 Humble, Gazebo, essential tools, and common dependencies:

```bash
 cd ~
 git clone https://github.com/ShivVary/CV-ROSWorkshop.git
 cd ~/CV-ROSWorkshop/"Installation Files"
 chmod +x install_combo.sh
 ./install_combo.sh
 source ~/.bashrc
 ```
 ### Test Run
 ```bash
 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
 ```