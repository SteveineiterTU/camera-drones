#!/bin/bash

# PARAMETERS
SESSION_NAME="ass_1"
TMUX='byobu' #Options: tmux | byobu
SOURCE='/home/student/.bashrc'

# General
cd /home/student/camera-drones

# Script
${TMUX} has-session -t ${SESSION_NAME}

if [ $? != 0 ]
then
  # Create the session
  ${TMUX} new-session -s ${SESSION_NAME} -n core -d
  sleep 1s

  # First window (0) -- start roscore
  ${TMUX} send-keys -t ${SESSION_NAME}:0 'roscore' C-m
  sleep 1s  # Needed, so that master is up to eg set rosparams

  # Nodes (1)  -- start nodes used in ass 1
  # Set up windows needed
  ${TMUX} new-window -n nodes -t ${SESSION_NAME}
  ${TMUX} split-window -v -t ${SESSION_NAME}:1
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.0
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.2

  #    astra camera
  ${TMUX} send-keys -t ${SESSION_NAME}:1.0 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.0 'roslaunch astra_camera astra.launch load_driver:=false publish_tf:=true --wait' C-m
  #    octomap server 
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'roslaunch octomap_server octomap_mapping.launch --wait' C-m
  #    rviz
  ${TMUX} send-keys -t ${SESSION_NAME}:1.2 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.2 'rosrun rviz rviz' C-m
  #   rosbag play
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'rosparam set /use_sim_time true' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'rosbag play --clock --rate=0.25 $HOME/camera-drones/assignment_1/2019-11-18-19-37-10.bag' C-m
  # Select bag window so we can pause it with space
  ${TMUX} select-window -t ${SESSION_NAME}:1.3
fi
${TMUX} attach -t ${SESSION_NAME}
