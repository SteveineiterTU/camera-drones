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
  #   talker
  ${TMUX} new-window -n nodes -t ${SESSION_NAME}
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1 'rosrun roscpp_tutorials talker' C-m
  #   listener
  ${TMUX} split-window -v -t ${SESSION_NAME}:1
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'rosrun roscpp_tutorials listener' C-m
  #   ros topic echo
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.0
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'rostopic echo /chatter' C-m

  ${TMUX} select-window -t ${SESSION_NAME}:1.0
fi
${TMUX} attach -t ${SESSION_NAME}
#byobu attach -t my_session
