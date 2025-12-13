#!/usr/bin/env bash

# =============================================================
# TMUX RUN SCRIPT FOR ROS1 MECANUM ROBOT (INTERACTIVE)
# =============================================================

SESSION="mecanum"

clear
echo "=============================================="
echo " ROS1 MECANUM ROBOT TMUX LAUNCHER"
echo "=============================================="
echo ""
echo "Select mode:"
echo "  1) Mapping (SLAM + Teleop)"
echo "  2) Navigation (AMCL + move_base)"
echo ""
read -p "Enter choice [1-2]: " CHOICE

if [[ "$CHOICE" != "1" && "$CHOICE" != "2" ]]; then
  echo "Invalid choice"
  exit 1
fi

# Kill existing session if exists
tmux has-session -t $SESSION 2>/dev/null
if [ $? == 0 ]; then
  tmux kill-session -t $SESSION
fi

# Create tmux session
tmux new-session -d -s $SESSION

# -------------------------------------------------------------
# Window 0 : roscore
# -------------------------------------------------------------
tmux rename-window -t $SESSION:0 roscore
tmux send-keys -t $SESSION:0 "roscore" C-m
sleep 2

# -------------------------------------------------------------
# Window 1 : LiDAR
# -------------------------------------------------------------
tmux new-window -t $SESSION -n lidar
tmux send-keys -t $SESSION:1 "roslaunch rplidar_ros rplidar.launch" C-m
sleep 2

# -------------------------------------------------------------
# Window 2 : Odometry
# -------------------------------------------------------------
tmux new-window -t $SESSION -n odom
tmux send-keys -t $SESSION:2 "rosrun mecanum_base_controller mecanum_odometry.py" C-m
sleep 1

# -------------------------------------------------------------
# Window 3 : Static TF
# -------------------------------------------------------------
tmux new-window -t $SESSION -n static_tf
tmux send-keys -t $SESSION:3 "roslaunch mecanum_base_controller static_tf.launch" C-m
sleep 1

# =============================================================
# MODE-SPECIFIC WINDOWS
# =============================================================

if [ "$CHOICE" == "1" ]; then
  
  # -----------------------------------------------------------
  # Window 4 : GMapping
  # -----------------------------------------------------------
  tmux new-window -t $SESSION -n gmapping
  tmux send-keys -t $SESSION:4 "roslaunch mecanum_base_controller gmapping.launch" C-m

  # -----------------------------------------------------------
  # Window 5 : Teleop
  # -----------------------------------------------------------
  tmux new-window -t $SESSION -n teleop
  tmux send-keys -t $SESSION:5 "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" C-m

elif [ "$CHOICE" == "2" ]; then

  # -----------------------------------------------------------
  # Window 4 : cmd_vel → wheels
  # -----------------------------------------------------------
  tmux new-window -t $SESSION -n cmd_vel
  tmux send-keys -t $SESSION:4 "rosrun mecanum_base_controller cmd_vel_to_wheels.py" C-m
  sleep 1

  # -----------------------------------------------------------
  # Window 5 : AMCL
  # -----------------------------------------------------------
  tmux new-window -t $SESSION -n amcl
  tmux send-keys -t $SESSION:5 "roslaunch mecanum_base_controller amcl.launch" C-m
  sleep 1

  # -----------------------------------------------------------
  # Window 6 : move_base
  # -----------------------------------------------------------
  tmux new-window -t $SESSION -n move_base
  tmux send-keys -t $SESSION:6 "roslaunch mecanum_base_controller move_base.launch" C-m
fi

# -------------------------------------------------------------
# Final Window : RViz
# -------------------------------------------------------------
tmux new-window -t $SESSION -n rviz
tmux send-keys -t $SESSION:7 "rviz" C-m

# Attach to tmux session
tmux attach -t $SESSION
