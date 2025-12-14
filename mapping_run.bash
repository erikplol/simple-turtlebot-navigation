#!/usr/bin/env bash
SESSION="mecanum_mapping"

tmux kill-session -t $SESSION 2>/dev/null
tmux new-session -d -s $SESSION -n "all_in_one"

# Split panes
tmux split-window -h -t $SESSION:0.0
tmux split-window -h -t $SESSION:0.1
tmux split-window -h -t $SESSION:0.2
tmux split-window -h -t $SESSION:0.3
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.4

tmux select-layout -t $SESSION:0 tiled

# Pane 0: roscore
tmux send-keys -t $SESSION:0.0 "source ~/simple-turtlebot-navigation/scripts/run_roscore.sh" C-m
sleep 10

# Pane 1: rplidar
tmux send-keys -t $SESSION:0.1 "source ~/simple-turtlebot-navigation/scripts/run_rplidar.sh" C-m
sleep 10

# Pane 2: odometry
tmux send-keys -t $SESSION:0.2 "source ~/simple-turtlebot-navigation/scripts/run_odometry.sh" C-m
sleep 10

# Pane 3: static_tf
tmux send-keys -t $SESSION:0.3 "source ~/simple-turtlebot-navigation/scripts/run_static_tf.sh" C-m
sleep 10

# Pane 4: gmapping
tmux send-keys -t $SESSION:0.4 "source ~/simple-turtlebot-navigation/scripts/run_gmapping.sh" C-m
sleep 10

# Pane 5: cmd_vel
tmux send-keys -t $SESSION:0.5 "source ~/simple-turtlebot-navigation/scripts/run_cmd_vel.sh" C-m
sleep 10

# Pane 6: move_base
tmux send-keys -t $SESSION:0.6 "source ~/simple-turtlebot-navigation/scripts/run_move_base.sh" C-m
sleep 10

# Pane 7 & 8 idle / monitor
tmux send-keys -t $SESSION:0.7 "source ~/simple-turtlebot-navigation/scripts/run_stm.sh" C-m
tmux send-keys -t $SESSION:0.8 "echo 'Idle Pane 8 – monitoring or rqt'" C-m

tmux attach -t $SESSION
