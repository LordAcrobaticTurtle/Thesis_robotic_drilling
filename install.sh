#!/bin/bash

# Install URsim
echo URsim must be installed before running this script.
echo Have you installed URsim?
read -p 'N: ' answer
echo Your answer was $answer

# echo "Edit script if you're ready to rumble."
# echo Exiting
# exit

# if [ "$answer" == "N" ]; then
# fi

echo Continuing with installation

# Install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y # if you haven't already installed curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update -y
sudo apt upgrade -y
sudo apt install ros-melodic-desktop-full -y
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
sudo apt install python-rosdep -y
sudo rosdep init -y
rosdep update


# Install ROS Control
sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers -y

#Install Moveit
sudo apt install ros-melodic-moveit -y 

# Install moveit visual tools
sudo apt-get install ros-melodic-moveit-visual-tools -y


echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc


mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone -b calibration_devel https://github.com/fmauch/universal_robot.git fmauch_universal_robot
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git Universal_Robots_ROS_Driver
git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel
git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel
git clone git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git


rosdep install -y --from-paths . --ignore-src --rosdistro melodic

sudo apt-get install ros-melodic-moveit-visual-tools -y
# sudo apt-get install ros-melodic-rqt-joint-trajectory-controller
sudo apt-get install ros-melodic-rqt-controller-manager -y

sudo apt install snapd
sudo snap install code --classic
echo 'export PATH=$PATH:/snap/bin' >> ~/.bashrc

# cd ~/catkin_ws
# catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
# catkin build


