#!/bin/bash
catkin_make
source devel/setup.zsh
# icp odometry. Remember to press space to start.
roslaunch icp_odometry icp.laun