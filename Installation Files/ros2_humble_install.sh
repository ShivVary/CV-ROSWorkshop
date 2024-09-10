
locale  # check for UTF-8

yes | sudo apt update && yes | sudo apt install locales
yes | sudo locale-gen en_US en_US.UTF-8
yes | sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


yes | sudo apt install software-properties-common
yes | sudo add-apt-repository universe

yes | sudo apt update && yes | sudo apt install curl -y
yes | sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

yes | sudo apt update

yes | sudo apt upgrade

yes | sudo apt install ros-humble-desktop
yes | sudo apt install ros-dev-tools
