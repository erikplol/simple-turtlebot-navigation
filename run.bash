#!/usr/bin/env bash

# =============================================================
# TMUX RUN SCRIPT FOR ROS1 MECANUM ROBOT (INTERACTIVE + PANES)
# =============================================================

SESSION="mecanum"

clear
echo "=============================================="
echo " ROS1 MECANUM ROBOT TMUX LAUNCHER (PANES)"
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

# -------------------------------------------------------------
# Create session + base layout
# -------------------------------------------------------------
tmux new-session -d -s $SESSION -n core

# Pane layout:
# [ roscore | lidar ]
# [ odom    | static_tf ]

tmux send-keys -t $SESSION:0.0 "roscore" C-m
sleep 2

tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "roslaunch rplidar_ros rplidar.launch" C-m
sleep 2

tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 "rosrun mecanum_base_controller mecanum_odometry.py" C-m
sleep 1

tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.3 "roslaunch mecanum_base_controller static_tf.launch" C-m
sleep 1

# =============================================================
# MODE-SPECIFIC WINDOW
# =============================================================

tmux new-window -t $SESSION -n mode

if [ "$CHOICE" == "1" ]; then
  # Mapping + Navigation (WITHOUT AMCL)
  # Robot can be moved while mapping

  tmux send-keys -t $SESSION:1.0 "roslaunch mecanum_base_controller gmapping.launch" C-m
  sleep 1

  tmux split-window -h -t $SESSION:1
  tmux send-keys -t $SESSION:1.1 "rosrun teleop_twist_keyboard teleop_twist_keyboard.py" C-m
  sleep 1

  tmux split-window -v -t $SESSION:1.0
  tmux send-keys -t $SESSION:1.2 "rosrun mecanum_base_controller cmd_vel_to_wheels.py" C-m
  sleep 1

  tmux split-window -v -t $SESSION:1.1
  tmux send-keys -t $SESSION:1.3 "roslaunch mecanum_base_controller move_base.launch" C-m
  sleep 1

  tmux new-window -t $SESSION -n rviz
  tmux send-keys -t $SESSION:2 "rviz" C-m

elif [ "$CHOICE" == "2" ]; then
  # Navigation layout:
  # [ cmd_vel | amcl ]
  # [ move_base | rviz ]

  tmux send-keys -t $SESSION:1.0 "rosrun mecanum_base_controller cmd_vel_to_wheels.py" C-m
  sleep 1

  tmux split-window -h -t $SESSION:1
  tmux send-keys -t $SESSION:1.1 "roslaunch mecanum_base_controller amcl.launch" C-m
  sleep 1

  tmux split-window -v -t $SESSION:1.0
  tmux send-keys -t $SESSION:1.2 "roslaunch mecanum_base_controller move_base.launch" C-m

  tmux split-window -v -t $SESSION:1.1
  tmux send-keys -t $SESSION:1.3 "rviz" C-m
fi

# Arrange nicely
tmux select-layout -t $SESSION tiled

# Attach
tmux attach -t $SESSION
