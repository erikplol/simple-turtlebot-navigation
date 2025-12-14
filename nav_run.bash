#!/usr/bin/env bash
# navigation_run_tmux_seq.sh

SESSION="mecanum_navigation"

# ---------------- CLEANUP ----------------
tmux kill-session -t $SESSION 2>/dev/null

# ---------------- TMUX SESSION ----------------
tmux new-session -d -s $SESSION -n "all_in_one"

# Split into 9 panes (same layout as mapping)
tmux split-window -h -t $SESSION
tmux split-window -h -t $SESSION
tmux split-window -h -t $SESSION
tmux split-window -h -t $SESSION
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.4

tmux select-layout -t $SESSION:0 tiled

# ---------------- PANES (sequential) ----------------

# Pane 0: roscore
tmux send-keys -t $SESSION:0.0 "~/simple-turtlebot-navigation/scripts/run_roscore.sh" C-m
sleep 5  # wait for roscore

# Pane 1: rplidar
tmux send-keys -t $SESSION:0.1 "~/simple-turtlebot-navigation/scripts/run_rplidar.sh" C-m
sleep 5  # wait for lidar node

# Pane 2: odometry
tmux send-keys -t $SESSION:0.2 "~/simple-turtlebot-navigation/scripts/run_odometry.sh" C-m
sleep 3

# Pane 3: static TF
tmux send-keys -t $SESSION:0.3 "~/simple-turtlebot-navigation/scripts/run_static_tf.sh" C-m
sleep 2

# Navigation-specific nodes

# Pane 4: cmd_vel → wheels
tmux send-keys -t $SESSION:0.4 "~/simple-turtlebot-navigation/scripts/run_cmd_vel.sh" C-m
sleep 2

# Pane 5: AMCL
tmux send-keys -t $SESSION:0.5 "~/simple-turtlebot-navigation/scripts/run_amcl.sh" C-m
sleep 3  # give AMCL time to initialize

# Pane 6: move_base
tmux send-keys -t $SESSION:0.6 "~/simple-turtlebot-navigation/scripts/run_move_base.sh" C-m
sleep 2

# Pane 7: idle / monitor
tmux send-keys -t $SESSION:0.7 "rosrun map_server map_server ~/simple-turtlebot-navigation/map/map.yaml'" C-m

# Pane 8: idle / monitor
tmux send-keys -t $SESSION:0.8 "echo 'Idle Pane 8 – monitoring or rqt'" C-m

# ---------------- ATTACH ----------------
tmux attach -t $SESSION
