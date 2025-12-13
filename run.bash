#!/usr/bin/env bash

# =============================================================
# HEADLESS-SAFE TMUX LAUNCHER FOR ROS1 MECANUM ROBOT (RASPI)
# No RViz, no keyboard teleop, safe for SSH / systemd
# =============================================================

SESSION="mecanum"
ROS_DISTRO=noetic
CATKIN_WS=~/catkin_ws

# ------------------ ROS ENV ------------------
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f ${CATKIN_WS}/devel/setup.bash ] && source ${CATKIN_WS}/devel/setup.bash

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

clear
echo "=============================================="
echo " ROS1 MECANUM ROBOT (HEADLESS MODE)"
echo "=============================================="
echo ""
echo "Select mode:"
echo "  1) Mapping (gmapping + move_base, NO AMCL)"
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

# =============================================================
# CORE WINDOW
# =============================================================
tmux new-session -d -s $SESSION -n core

# Pane layout:
# [ roscore | rplidar ]
# [ odom    | static_tf ]

# roscore
tmux send-keys -t $SESSION:0.0 "roscore" C-m
sleep 2

# lidar
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "roslaunch rplidar_ros rplidar.launch" C-m
sleep 2

# odom
tmux split-window -v -t $SESSION:0.0
tmux send-keys -t $SESSION:0.2 "rosrun mecanum_base_controller mecanum_odometry.py" C-m
sleep 1

# static tf
tmux split-window -v -t $SESSION:0.1
tmux send-keys -t $SESSION:0.3 "roslaunch mecanum_base_controller static_tf.launch" C-m
sleep 1

# =============================================================
# MODE WINDOW
# =============================================================
tmux new-window -t $SESSION -n mode

if [ "$CHOICE" == "1" ]; then
  # Mapping + Navigation (NO AMCL)
  # [ gmapping | cmd_vel ]
  # [ move_base | idle ]

  tmux send-keys -t $SESSION:1.0 "roslaunch mecanum_base_controller gmapping.launch" C-m
  sleep 1

  tmux split-window -h -t $SESSION:1
  tmux send-keys -t $SESSION:1.1 "rosrun mecanum_base_controller cmd_vel_to_wheels.py" C-m
  sleep 1

  tmux split-window -v -t $SESSION:1.0
  tmux send-keys -t $SESSION:1.2 "roslaunch mecanum_base_controller move_base.launch" C-m

elif [ "$CHOICE" == "2" ]; then
  # Navigation
  # [ cmd_vel | amcl ]
  # [ move_base | idle ]

  tmux send-keys -t $SESSION:1.0 "rosrun mecanum_base_controller cmd_vel_to_wheels.py" C-m
  sleep 1

  tmux split-window -h -t $SESSION:1
  tmux send-keys -t $SESSION:1.1 "roslaunch mecanum_base_controller amcl.launch" C-m
  sleep 1

  tmux split-window -v -t $SESSION:1.0
  tmux send-keys -t $SESSION:1.2 "roslaunch mecanum_base_controller move_base.launch" C-m
fi

# Tidy layout
tmux select-layout -t $SESSION tiled

# Attach
tmux attach -t $SESSION
