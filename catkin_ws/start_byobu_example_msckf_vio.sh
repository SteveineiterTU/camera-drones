#!/bin/bash

# PARAMETERS
SESSION_NAME="my_session"
TMUX='byobu' #Options: tmux | byobu
SOURCE='/home/student/.bashrc'

# General
cd /home/student

# Script
${TMUX} has-session -t ${SESSION_NAME}

if [ $? != 0 ]
then
  # Create the session
  ${TMUX} new-session -s ${SESSION_NAME} -n core -d

  # First window (0) -- start roscore
  ${TMUX} send-keys -t ${SESSION_NAME}:0 'roscore' C-m
  sleep 2s

  # Nodes (1)
  #   msckf_vio
  ${TMUX} new-window -n nodes -t ${SESSION_NAME}
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'rosparam set /use_sim_time true' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'roslaunch msckf_vio msckf_vio_fla.launch --wait' C-m
  sleep 2
  #   rviz
  ${TMUX} split-window -v -t ${SESSION_NAME}:1
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'roslaunch msckf_vio rviz.launch --wait' C-m
  #   rosbag play - open sub-terminal
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.0
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source $HOME/.bashrc' C-m
  #   rqt_multiplot
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.2
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'roslaunch msckf_vio rqt_multiplot.launch --wait' C-m
  #  rosbag play - run node
  sleep 10s
  #${TMUX} send-keys -t ${SESSION_NAME}:1.1 'rosbag  play /media/student_DATA/workspaces/ros/dl_19_20_ws/bags/fla_wg_175.bag' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'roslaunch msckf_vio rosbag_play.launch' C-m

  ${TMUX} select-window -t ${SESSION_NAME}:1.0
fi
${TMUX} attach -t ${SESSION_NAME}
#byobu attach -t my_session



