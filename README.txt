This project demonstrates a fully autonomous TurtleBot3 in a simulated home environment using ROS2 Humble, Gazebo, Nav2, and a Machine Learning decision system.
The robot maps the house, localizes itself, receives task conditions, predicts the correct room to visit, and autonomously navigates there.

🚀 How to Run the Project
🟣 1. Start Gazebo Simulation
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

🟢 2. Launch RViz + Nav2
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=$HOME/house_map_new.yaml

🔵 3. Launch the Autonomous System (ML + Navigation Nodes)
cd ~/ROS_project
source install/setup.bash
ros2 launch Turtlebot_proj system.launch.py

📡 Useful Debug Commands

Run these in a separate terminal:
ros2 node list
ros2 topic list
ros2 topic echo /task_conditions
ros2 topic echo /target_room
ros2 topic echo /amcl_pose

🧠 Tech Highlights
ROS2 Humble + Gazebo Simulation
Nav2 (AMCL, global/local planners, behavior trees)
Custom ROS2 package with 3 nodes (input, decision, navigator)
Machine Learning model (DecisionTreeClassifier) for autonomous room selection
RViz2 visualization + TF2 transforms

Video Demo- https://youtu.be/vlXa4luMUeE
2D Map generation video- https://youtu.be/_MXygqUz-Uw 
