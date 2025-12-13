#!/usr/bin/env bash
# run_move_base.sh
source /opt/ros/noetic/setup.bash
source ~/simple-turtlebot-navigation/devel/setup.bash
roslaunch mecanum_base_controller move_base.launch
