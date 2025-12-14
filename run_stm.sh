#!/usr/bin/env bash
# run_odometry.sh
source /opt/ros/noetic/setup.bash
source ~/simple-turtlebot-navigation/devel/setup.bash
rosrun mecanum_base_controller stm_node.py

