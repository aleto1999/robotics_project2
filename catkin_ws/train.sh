#!/bin/bash
catkin_make
source /opt/ros/melodic/setup.zsh
source ~/Stingray-Simulation/catkin_ws/devel/setup.zsh
source ~/Stingray-Simulation/stingray_setup.bash
roslaunch stingray_sim wall_following_v1.launch

