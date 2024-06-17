#!/bin/bash

tmux set -g mouse on

SESSION="drone"


# 1. Start a new session (detached)
tmux new-session -d -s $SESSION

tmux source-file .tmux.conf

# 2. Create the horizontal split (2 rows)
tmux split-window -v -t $SESSION

# 3. Split the top row into 3 columns
tmux select-pane -t 0
tmux split-window -h -t $SESSION
tmux select-pane -t 0
tmux split-window -h -t $SESSION

# 4. Split the bottom row into 3 columns 
# (Note: pane indices shift as you create them)
tmux select-pane -t 3
tmux split-window -h -t $SESSION
tmux select-pane -t 3
tmux split-window -h -t $SESSION

# 5. Populate panes with text (but don't press Enter)
# -t specifies the pane index, the string is the command
tmux send-keys -t 0 "ros2 launch px4_autonomy_modules mavros.launch.py gcs_url:=udp://@"
tmux send-keys -t 1 "ros2 launch realsense2_camera rs_launch.py"
tmux send-keys -t 2 "ros2 launch vision_bdg vision_bdg.launch.py use_vicon:=false use_realsense:=true"
tmux send-keys -t 3 "ros2 run flight3 comm_node"
tmux send-keys -t 4 "ros2 service call /rob498_drone_8/comm/launch std_srvs/srv/Trigger {}"
tmux send-keys -t 5 "ros2 service call /rob498_drone_8/comm/test std_srvs/srv/Trigger {}"

# 6. Attach to the session
tmux attach-session -t $SESSION
