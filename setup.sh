#!/bin/bash
set -e

SESSION="drone"
GCS_IP="10.42.0.123"

# kill old session if it exists
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

# 1. Start a new session (detached)
tmux new-session -d -s "$SESSION"

# 2. Now it's safe to set tmux options
tmux set -g mouse on

tmux source-file .tmux.conf

# 3. Create 3 rows total
tmux split-window -v -t "$SESSION"
tmux select-pane -t 1
tmux split-window -v -t "$SESSION"

# 4. Split the top row into 4 columns
tmux select-pane -t 0
tmux split-window -h -t "$SESSION"
tmux select-pane -t 0
tmux split-window -h -t "$SESSION"
tmux select-pane -t 0
tmux split-window -h -t "$SESSION"

# 5. Split the middle row into 4 columns
tmux select-pane -t 4
tmux split-window -h -t "$SESSION"
tmux select-pane -t 4
tmux split-window -h -t "$SESSION"
tmux select-pane -t 4
tmux split-window -h -t "$SESSION"

# 6. Split the bottom row into 4 columns
tmux select-pane -t 8
tmux split-window -h -t "$SESSION"
tmux select-pane -t 8
tmux split-window -h -t "$SESSION"
tmux select-pane -t 8
tmux split-window -h -t "$SESSION"

# 7. Populate panes with text (but don't press Enter)
tmux send-keys -t 0  "ros2 launch px4_autonomy_modules mavros.launch.py gcs_url:=udp://@$GCS_IP"
tmux send-keys -t 1  "ros2 launch realsense2_camera rs_launch.py"
tmux send-keys -t 2  "ros2 launch vision_bdg vision_bdg.launch.py use_vicon:=true use_realsense:=false"
tmux send-keys -t 3  "ros2 run perception depth_node"
tmux send-keys -t 4  "ros2 run comm occupancy_node"
tmux send-keys -t 5  "ros2 run comm local_planner_node"
tmux send-keys -t 6  "ros2 run comm comm_node"
tmux send-keys -t 7  "ros2 run waypoint_publisher waypoint_publisher"
tmux send-keys -t 8  "ros2 service call /rob498_drone_8/comm/launch std_srvs/srv/Trigger {}"
tmux send-keys -t 9  "ros2 service call /rob498_drone_8/comm/test std_srvs/srv/Trigger {}"
tmux send-keys -t 10  "ros2 service call /rob498_drone_8/comm/abort std_srvs/srv/Trigger {}"
tmux send-keys -t 11  ros2 bag record -a
#tmux send-keys -t 10 "./video.sh \"$GCS_IP\""
# pane 11 intentionally left blank

# 8. Attach
tmux attach-session -t "$SESSION"