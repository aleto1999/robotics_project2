#!/bin/bash
catkin_make
source /opt/ros/melodic/setup.bash
source ~/Stingray-Simulation/catkin_ws/devel/setup.bash
source ~/Stingray-Simulation/stingray_setup.bash
roslaunch stingray_sim run_sim.launch

