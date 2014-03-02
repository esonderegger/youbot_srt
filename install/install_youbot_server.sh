#!/usr/bin/env bash
# install_youbot.sh
# run this to install necessary packages onto the Kuka install_youbot

sudo apt-get install ros-hydro-octomap-rviz-plugins ros-hydro-octomap-mapping ros-hydro-orocos-kdl  ros-hydro-joy ros-hydro-moveit-full

cd ~/catkin_ws/src

git clone https://github.com/WPI-RAIL/brics_actuator.git
git clone -b hydro-devel https://github.com/ros-drivers/hokuyo_node.git
git clone -b hydro-devel https://github.com/ros-planning/moveit_commander.git
git clone https://github.com/esonderegger/youbot_description.git
git clone https://github.com/esonderegger/youbot_common.git
git clone https://github.com/esonderegger/youbot_driver.git
got clone https://github.com/esonderegger/youbot_navigation.git
