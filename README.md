# VM Setup links
## GCP
```
https://console.cloud.google.com/compute
```
## Remote connect to desktop
```
https://remotedesktop.google.com/headless
```
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
## Install packages in the VM
```
cd ~
git clone https://github.com/ShivVary/ROSWorkshop.git
cd ~/ROSWorkshop/"Installation Files"
chmod +x install_combo.sh
./install_combo.sh
source ~/.bashrc
```
## Test Run in the VM
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
