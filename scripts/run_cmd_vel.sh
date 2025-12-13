#!/usr/bin/env bash
# run_cmd_vel.sh
source /opt/ros/noetic/setup.bash
source ~/simple-turtlebot-navigation/devel/setup.bash
rosrun mecanum_base_controller cmd_vel_to_wheels.py

