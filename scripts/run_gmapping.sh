#!/usr/bin/env bash
# run_gmapping.sh
source /opt/ros/noetic/setup.bash
source ~/simple-turtlebot-navigation/devel/setup.bash
roslaunch mecanum_base_controller gmapping.launch
