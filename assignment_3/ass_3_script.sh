#!/bin/bash

# PARAMETERS
SESSION_NAME="ass_3"
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
  sleep 1s

  # First window (0) -- start roscore
  ${TMUX} send-keys -t ${SESSION_NAME}:0 'roscore' C-m
  sleep 1s  # Needed, so that master is up to eg set rosparams

  # Nodes (1)  -- start nodes used in ass 1
  # Set up windows needed
  ${TMUX} new-window -n nodes_passiv -t ${SESSION_NAME}
  ${TMUX} split-window -v -t ${SESSION_NAME}:1
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.0
  ${TMUX} split-window -h -t ${SESSION_NAME}:1.2
  ${TMUX} split-window -v -t ${SESSION_NAME}:1.2
  ${TMUX} split-window -v -t ${SESSION_NAME}:1.4

  ${TMUX} new-window -n acitve_stuff -t ${SESSION_NAME}
  ${TMUX} split-window -v -t ${SESSION_NAME}:2
  ${TMUX} split-window -h -t ${SESSION_NAME}:2.0
  ${TMUX} split-window -h -t ${SESSION_NAME}:2.2

     # Nodes passive
  #    Path planner ros node
  ${TMUX} send-keys -t ${SESSION_NAME}:1.0 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.0 'rosrun octomap_path_planner octomap_path_planner_ros_node trajectory_visualization --runtime 3 --planner RRTStar -o WeightedLengthAndClearanceCombo -f planner_trajectory.txt --info 2 --octomap /home/student/camera-drones/catkin_ws/src/octomap_path_planner/maps/power_plant.bt' C-m
  #    Octomap server 
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.1 'roslaunch octomap_path_planner octomap_mapping_a2.launch octomap:=$HOME/camera-drones/catkin_ws/src/octomap_path_planner/maps/power_plant.bt --wait' C-m
  #    Trajectory Sampler
  ${TMUX} send-keys -t ${SESSION_NAME}:1.2 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.2 'roslaunch trajectory_planner trajectory_planner.launch dt:=0.01' C-m  
  #    Set a local transform so we can use world.
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.3 'rosrun tf static_transform_publisher 0 0 0 0 0 0 1 world my_frame 10' C-m
  #   Trajectory visualization
  ${TMUX} send-keys -t ${SESSION_NAME}:1.4 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.4 'rosrun octomap_path_planner octomap_path_planner_trajectory_visualization' C-m
  # Rviz (Set marker and occupancy Grid with binary in rviz)
  ${TMUX} send-keys -t ${SESSION_NAME}:1.5 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:1.5 'rosrun rviz rviz' C-m

    # Active stuff
  # Send keys (see README in octomap path planner package for nice points)
  ${TMUX} send-keys -t ${SESSION_NAME}:2.0 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:2.0 'rostopic pub /octomap_path_planner/current_position geometry_msgs/Point "x: 14
y: 8
z: 5" --once' C-m
  sleep 4s
  ${TMUX} send-keys -t ${SESSION_NAME}:2.2 'source $HOME/.bashrc' C-m
  ${TMUX} send-keys -t ${SESSION_NAME}:2.2 'rostopic pub /octomap_path_planner/goal_position geometry_msgs/Point "x: 14
y: 8
z: 20" --once' C-m
  sleep 4s

  # Select unused window so we can debug (eg rostopic, rosnode, ..)
  ${TMUX} select-window -t ${SESSION_NAME}:2.1
fi
${TMUX} attach -t ${SESSION_NAME}
