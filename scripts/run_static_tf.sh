#!/usr/bin/env bash
# run_static_tf.sh
source /opt/ros/noetic/setup.bash
source ~/simple-turtlebot-navigation/devel/setup.bash
roslaunch mecanum_base_controller static_tf.launch
