#!/bin/bash
catkin_make
source ./devel/setup.zsh
roslaunch icp_odometry icp.launch