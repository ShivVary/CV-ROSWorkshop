# OpenCV + ROS2 Tutorial
---
#  Commands for enviroment setup
## VM account setup
```
sudo apt upgrade && sudo apt update 
sudo usermod -aG sudo <username>
sudo passwd <username>
```
## Setup remote desktop
```
wget https://dl.google.com/linux/direct/chrome-remote-desktop_current_amd64.deb 
sudo apt-get install --assume-yes ./chrome-remote-desktop_current_amd64.deb
sudo apt update && sudo apt upgrade
sudo apt install slim
sudo apt install ubuntu-desktop
```
## Restart slim 
```
sudo reboot
sudo service slim start
```
## Install packages
```
cd ~
git clone https://github.com/ShivVary/ROSWorkshop.git
cd ~/ROSWorkshop/"Installation Files"
chmod +x install_combo.sh
./install_combo.sh
```
## Test Run
```
source ~/.bashrc
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
---
## Day 1 - OpenCV tutorial 
Link to the [workshop tutorial](https://docs.google.com/document/d/1BChFJIBk2sLydrGdqkoIz4FfSqWrsZOTjOvE-tvFalQ/edit?usp=sharing)
## What is OpenCV?
OpenCV (Open Source Computer Vision Library) is an open-source software library that provides a vast range of computer vision and machine learning algorithms. It is widely used in both academic and industrial settings for developing vision-based applications. Here are some key benefits of using OpenCV:

- Comprehensive Library of Algorithms: OpenCV provides a large collection of algorithms for image processing, object detection, facial recognition, feature extraction, and more, making it versatile for various computer vision tasks.

- Multi-Language Support: OpenCV supports multiple programming languages, including C++, Python, Java, and more, allowing developers to use the language they are most comfortable with.

- Cross-Platform Compatibility: OpenCV works on various operating systems, including Windows, Linux, macOS, Android, and iOS, making it adaptable for both desktop and mobile applications.

- Real-Time Processing: OpenCV is optimized for real-time applications, enabling high-speed performance essential for tasks like video processing, object tracking, and real-time monitoring systems.

- Hardware Acceleration: OpenCV can leverage hardware acceleration through CUDA and OpenCL, allowing it to run efficiently on GPUs for demanding computational tasks.

- Extensive Community and Documentation: OpenCV has a large, active community that contributes to continuous updates, improvements, and extensive documentation, making it easier to find support, tutorials, and resources.

- Integration with Machine Learning Libraries: OpenCV can be easily integrated with popular machine learning libraries such as TensorFlow, PyTorch, and Scikit-learn, enhancing its capabilities for deep learning applications.

- Open-Source and Free to Use: OpenCV is open-source and free, making it accessible for anyone looking to develop computer vision applications without the cost constraints of commercial software.

OpenCV is widely adopted in fields such as robotics, autonomous driving, augmented reality, and surveillance due to its powerful features, flexibility, and the ability to handle a broad range of vision-based tasks efficiently.

---
## Day 2 - ROS2 tutorial 
--- 
## What is ROS 2? Why should I use it?
ROS 2 (Robot Operating System 2) is the next generation of the original ROS, designed to address the limitations of ROS 1 and make it more suitable for modern robotics applications. Here are some key benefits of using ROS 2:

- Enhanced Modularity: Similar to ROS 1, ROS 2 allows you to develop different robot components separately, such as sensing, control, and actuation systems, which helps manage complexity.
- Multi-Language Support: ROS 2 supports C++, Python, and other languages, allowing you to leverage the strengths of each.
- Real-Time Capabilities: ROS 2 is designed to support real-time systems, making it suitable for critical robotics applications that require precise timing and control.
- Improved Security: ROS 2 incorporates better security features, which are essential for robotics applications in sensitive environments.
- Support for DDS (Data Distribution Service): DDS is the backbone of ROS 2, allowing for efficient communication across different platforms and devices, including distributed systems.
- Cross-Platform Support: ROS 2 works on Linux and Windows allowing developers to choose their preferred operating system.

ROS 2 is widely used in industry and research because of its flexibility, scalability, and ability to integrate with modern robotic hardware and software.

# Tutorial Requirements
To comfortably understand this tutorial, you should be familiar with basic command line operations, including navigating directories with cd and executing commands. Knowledge of package management and basic software development principles will also be beneficial.

## Optional but helpful skills include:
- Solid Programming Skills: Proficiency in Python will make it easier to develop ROS 2 nodes and debug code.
- Basic Networking Knowledge: Understanding how networks work will help when working with distributed ROS 2 systems.
- Stable Internet Connection: Essential for downloading packages, updates, and dependencies.
  
You can run ROS 2 directly on Linux or Windows, or you can use Docker for a more isolated environment.To save time, we are setting up a VM hosted on GCP. Installation guides are available on the official ROS 2 website, and you can find detailed instructions for your operating system there.

## More Resources
ROS 2 includes powerful features like actions, nodes, topics, services, parameters, logging, and the ability to operate across multiple devices. To learn more about these capabilities, the official ROS 2 documentation and tutorials are a great place to start: https://docs.ros.org/en/humble/index.html. Happy coding with ROS 2!

---
