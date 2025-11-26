Main Build folder name - ROS_project

Dataset used - provided

RUN Commands

#run gazebo
terminal 1 - source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

#run Rviz and NAV2
terminal 2 - source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/house_map_new.yaml

#run lunch file and Nodes
terminal 3 - cd ~/ROS_project
source install/setup.bash
ros2 launch Turtlebot_proj system.launch.py

#To check status and data from nodes
In seperate terminal open -
ros2 node list
ros2 topic list
ros2 topic echo /task_conditions
ros2 topic echo /target_room
ros2 topic echo /amcl_pose

