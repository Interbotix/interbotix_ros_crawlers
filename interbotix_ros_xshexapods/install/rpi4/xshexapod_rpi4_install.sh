#!/usr/bin/env bash

ubuntu_version="$(lsb_release -r -s)"

if [ $ubuntu_version == "18.04" ]; then
  ROS_NAME="melodic"
else
  echo -e "Unsupported Ubuntu verison: $ubuntu_version"
  echo -e "Interbotix Hexapod only works with 18.04 on the Raspberry Pi"
  exit 1
fi

read -p "What is your robot model? (ex. wxmark4): " ROBOT_MODEL
read -p "Run the Joystick ROS package at system boot? " resp
if [[ $resp == [yY] || $resp == [yY][eE][sS] ]]; then
  run_joy_at_boot=true
else
  run_joy_at_boot=false
fi

echo "Ubuntu $ubuntu_version detected. ROS-$ROS_NAME chosen for installation.";

echo -e "\e[1;33m ******************************************** \e[0m"
echo -e "\e[1;33m The installation may take around 15 Minutes! \e[0m"
echo -e "\e[1;33m ******************************************** \e[0m"
sleep 4
start_time="$(date -u +%s)"

# Install/Configure some basic packages
sudo apt update && sudo apt -y upgrade
sudo apt -y autoremove
sudo apt -y install python-pip
sudo -H pip install rpi_ws281x

# Install ROS
# Step 1: Install ROS
if [ $(dpkg-query -W -f='${Status}' ros-$ROS_NAME-desktop-full 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Installing ROS..."
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
  sudo apt update
  sudo apt -y install ros-$ROS_NAME-desktop-full
  if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
  fi
  echo "source /opt/ros/$ROS_NAME/setup.bash" >> ~/.bashrc
  sudo apt -y install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
  sudo rosdep init
  rosdep update
else
  echo "ros-$ROS_NAME-desktop-full is already installed!"
fi
source /opt/ros/$ROS_NAME/setup.bash

# Step 2: Install Hexapod packages
INTERBOTIX_WS=~/interbotix_ws
if [ ! -d "$INTERBOTIX_WS/src" ]; then
  echo "Installing ROS packages for the Interbotix Hexapod..."
  mkdir -p $INTERBOTIX_WS/src
  cd $INTERBOTIX_WS/src
  git clone https://github.com/Interbotix/interbotix_ros_core.git
  git clone https://github.com/Interbotix/interbotix_ros_crawlers.git
  cd interbotix_ros_crawlers
  git checkout $ROS_NAME
  cd ..
  git clone https://github.com/Interbotix/interbotix_ros_toolboxes.git
  cd interbotix_ros_toolboxes/interboix_xs_toolbox
  touch interbotix_xs_moveit_interface/CATKIN_IGNORE
  touch interbotix_xs_ros_control/CATKIN_IGNORE
  cd $INTERBOTIX_WS/src/interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
  sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  cd $INTERBOTIX_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  echo "source $INTERBOTIX_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "Interbotix Hexapod ROS packages already installed!"
fi
source $INTERBOTIX_WS/devel/setup.bash

# Step 3: Setup Environment Variables
if [ -z "$ROS_IP" ]; then
  echo "Setting up Environment Variables..."
  echo "export ROBOT_MODEL=$ROBOT_MODEL" >> ~/.bashrc
  echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
  echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
else
  echo "Environment variables already set!"
fi

# Step 4: Configure 'run at startup' feature
if [ "$run_joy_at_boot" = true ]; then
  cd $INTERBOTIX_WS/src/interbotix_ros_crawlers/interbotix_ros_xshexapods/install/rpi4/
  touch xshexapod_rpi4_launch.sh
  echo -e "#!/usr/bin/env bash

# This script is called by the xshexapod_rpi4_boot.service file when
# the Raspberry Pi boots. It just sources the ROS related workspaces
# and launches the xshexapod_joy launch file. It is populated with the correct commands
# from the xshexapod_rpi4_install.sh installation script.

source /opt/ros/$ROS_NAME/setup.bash
source $INTERBOTIX_WS/devel/setup.bash
roslaunch interbotix_xshexapod_joy xshexapod_joy.launch use_rviz:=false robot_model:=$ROBOT_MODEL" > xshexapod_rpi4_launch.sh

  chmod +x xshexapod_rpi4_launch.sh
  sudo cp xshexapod_rpi4_boot.service /lib/systemd/system/
  sudo systemctl daemon-reload
  sudo systemctl enable xshexapod_rpi4_boot.service
fi

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo "Installation complete, took $elapsed seconds in total"
echo "NOTE: Remember to reboot the computer before using the robot!"
