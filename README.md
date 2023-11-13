# camera-drones

## Commands from Lecutre:
- rqt_bag /\<bag name> (to inspect bags)
  
### Byobu
- shift + F1 help
- shift + arrows navigate through window
- alt + arrows naviaget through workspaces
- alt + shift + arrows change shape of split
- F2 new window
- shift + F2 horizontal split
- ctrl + F2 vertical split

## From my Udemy Course:
### Ros commands:
- catkin_make
- catking_create_pkg \<name of package>
- **roslaunch** \<package name> \<launch file name>  
- roscore
- rosrun \<package> \<node name>
- rosrun rqt_graph rqt_graph  (visualization)
- rosnode (with -h get all funcitons)
  - list
  - info /\<node name>
  - ping /\<node name>
  - kill /\<node name>
- rostopic (with -h get all funcitons)
  - list
  - echo /\<node name>
  - pub (-r 5 for 5 hz, -1 for just one post) \<use autocomplete for rest xD>
- rosservice (with -h get all functions)
  - list
  - info
  - call /\<node name> \<press tab to create template of params>
- rosmsg (with -h get all functions)
  - list
  - show \<package\/message name>
- rossrv (with -h get all functions) (Not same as rosservice)
  - list
  - show \<package\/message name>
- rosparam (with -h get all functions)
  - list
  - set \<\/name of parameter> \<value of parameter>   
  - get \<\/name of parameter>
- rosbag (with -h get all functions) (nice for testing)
  - record \<name of topic>
  - info \<name of rosbag>
  - play \<name of rosbag>


### Good practices:
- ask yourself if a topic needs to be asynchon or not
- Services often used for calculaitons or to quickly call an action
- Have a seperate package for custom messages and services (to avoid dependency mess)
- Custom messages convention to have them PascalCase
- folder must match the name of CMakeLists (msg, srv,...)
- Have a seperate package for launch files (to avoid depenency mess)



