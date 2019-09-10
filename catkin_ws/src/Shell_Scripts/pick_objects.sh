#!/bin/sh

#Launch pick_objects node

#Launch turtlebot_world.launch to deploy a turtlebot in the environment.
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5

# #Customize world
#xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/#catkin_ws/src/world/winding_path.world " &
#sleep 5


#Launch amcl_demo.launch to localize the turtlebot.
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5

#Launch the view_navigation.launch to observe the map in Rviz.
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

#Launch pick_objects node.
xterm -e " rosrun pick_objects pick_objects " 
