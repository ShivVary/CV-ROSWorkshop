3# 🤖 ROS 2 Workshop

Welcome to the **ROS 2 Workshop** — a hands-on introduction to the Robot Operating System 2 by **University of Sydney Robotics Club**!  
Whether you're a beginner or an experienced developer exploring robotics, this workshop will guide you through the essentials of ROS 2 and give you practical experience in building robot applications.

---

## 🗂️ Contents

### 📘 Introduction
Overview of ROS 2, its ecosystem, and what you'll learn in this workshop.

### 🛠️ Day 1: ROS Introduction + Virtual Machine Setup
- Introduction to ROS concepts
- ROS 2 architecture and terminology
- Setting up a ROS 2 environment using a VM on VirtualBox

### 🚀 Day 2: Intermediate ROS + Mini Challenge
- Writing and launching ROS 2 nodes
- Topics, services, and parameter usage
- A hands-on challenge to apply what you’ve learned

---

## ❓ What is ROS 2? Why Should I Use It?

**ROS 2 (Robot Operating System 2)** is the next-generation robotics framework designed to overcome the limitations of ROS 1 and meet the demands of real-world, production-grade robotics systems.

### 🌟 Key Benefits:
- **🧩 Modularity** – Build systems with reusable components for control, sensing, and actuation
- **🛠️ Multi-Language Support** – Develop in C++, Python, and more
- **⏱️ Real-Time Support** – Critical for time-sensitive robotic applications
- **🔒 Improved Security** – Secure communication and node authentication
- **📡 DDS Middleware** – Efficient, scalable communication using the Data Distribution Service
- **🖥️ Cross-Platform** – Works on Linux, Windows, and supports Docker for containerized environments

ROS 2 is widely used across industry and academia due to its scalability, flexibility, and large ecosystem of tools and libraries.

---

## 🧰 Requirements

To get the most out of this workshop, you should be familiar with:

### ✅ Pre-requisites:
- Proficient in python **(Passed ENG1810/INFO1110)**
- Basic command-line usage (`cd`, `ls`, `bash` scripting, etc.)
- Software development principles and using the terminal

---

## 📅 [Day 1: ROS Introduction + VM Setup](Docs/Day1.md)

On Day 1, we’ll cover the foundational concepts of ROS 2 and walk through setting up your development environment locally using a pre-configured VM.

### 🧠 Topics Covered:
- What is ROS and how it works
- Core concepts: nodes, topics, services
- ROS 2 workspace structure and tooling
- Virtual Machine setup on Google Cloud Platform

---

## 📅 [Day 2: ROS 2 Intermediate – OpenCV, Gazebo, Actions & Services](Docs/Day2.md)

On Day 2, we build on the fundamentals and dive into **practical development** with ROS 2. You’ll write your own nodes, interface with Gazebo, integrate computer vision using OpenCV, and learn how to implement services and actions in a ROS 2 system.

### 🧠 Topics Covered:
- Writing custom ROS 2 nodes in Python
- Subscribing and publishing with image data using OpenCV
- Simulating robots and environments in Gazebo
- Implementing and calling services
- Understanding and using ROS 2 actions for long-running tasks

### 🧪 Hands-On Activities:
- Launch a Gazebo world and spawn a robotb
- Capture and process camera data in real time with OpenCV
- Create a service server and client in Python
- Trigger an action server for a robot to complete a navigation task

---

## 💻 Development Environment

To streamline setup and focus on learning, we’re using a [pre-configured Virtual Machine (VM) hosted on Virtual Box](Docs/Setup.md)

> Prefer to use your own setup?
[You can install Ubuntu 22.04 on bare metal and run ROS 2 locally](Docs/Setup.md)
  
Official installation instructions are available on the [ROS 2 documentation site](https://docs.ros.org/en/rolling/Installation.html).

--- 

## 📚 More Resources

Interested in exploring advanced features of ROS 2?  
The official [ROS 2 documentation and tutorials](https://docs.ros.org/en/humble/index.html) are an excellent place to dive deeper into its capabilities.

From real-time systems to robot navigation, simulation, and beyond — there's a lot to explore.

Happy coding with ROS 2! 🚀
---
