Testing instructions
--------------------
* To start the ROS node 

rosrun octomap_path_planner octomap_path_planner_ros_node --octomap /home/student/camera-drones/catkin_ws/src/octomap_path_planner/maps/power_plant.bt


* To change the initial/current position run:

3D (when using 3D version):
rostopic pub /octomap_path_planner/current_position geometry_msgs/Point "x: 0.1
y: 0.1
z: 0.1" --once


* To change the initial/goal position and plan a trajectory run:

3D (when using 3D version):
rostopic pub /octomap_path_planner/goal_position geometry_msgs/Point "x: 0.9
y: 0.9
z: 0.9" --once


* For  visualization run:
rosrun octomap_path_planner octomap_path_planner_trajectory_visualization

We also need a static tf for the world: 
  rosrun tf static_transform_publisher 0 0 0 0 0 0 1 world my_frame 10

then run:
  rviz

inside rviz add a "display" of typer "Marker" subscribed to the topic "/trajectory_visualization/trajectory_markers". Afterwards every new trajectory published by the planner should be visualized in rviz.

* To launch the octomap_server:

roslaunch dla2_path_planner octomap_mapping_a2.launch
