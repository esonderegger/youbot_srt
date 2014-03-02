#!/usr/bin/env bash
# install_youbot.sh
# run this to install necessary packages onto the Kuka install_youbot

cd ~/catkin_ws/src

git clone https://github.com/WPI-RAIL/brics_actuator.git
git clone -b hydro-devel https://github.com/ros-drivers/hokuyo_node.git
git clone https://github.com/esonderegger/youbot_description.git
git clone https://github.com/esonderegger/youbot_common.git
git clone https://github.com/esonderegger/youbot_driver.git