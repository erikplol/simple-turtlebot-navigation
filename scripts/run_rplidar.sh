#!/usr/bin/env bash
# run_rplidar.sh
source /opt/ros/noetic/setup.bash
source ~/simple-turtlebot-navigation/devel/setup.bash
roslaunch rplidar_ros rplidar_a2m8.launch
