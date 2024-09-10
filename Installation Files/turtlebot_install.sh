source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.bash
source ~/.bashrc

yes | sudo apt install ros-humble-gazebo-*
yes | sudo apt install ros-humble-cartographer
yes | sudo apt install ros-humble-cartographer-ros
yes | sudo apt install ros-humble-navigation2
yes | sudo apt install ros-humble-nav2-bringup

yes | sudo apt install ros-humble-dynamixel-sdk
yes | sudo apt install ros-humble-turtlebot3-msgs
yes | sudo apt install ros-humble-turtlebot3

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

source ~/turtlebot3_ws/install/setup.bash
. /usr/share/gazebo-11/setup.sh
cd ~/turtlebot3_ws && colcon build --symlink-install

sudo echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
sudo echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
sudo echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
sudo echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
