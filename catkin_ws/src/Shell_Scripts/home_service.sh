#!/bin/sh

#Launch home_service node

#Launch turtlebot_world.launch to deploy a turtlebot in the environment.
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5


#Launch amcl_demo.launch to localize the turtlebot.
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5

##Launch the view_navigation.launch to observe the map in Rviz.
#xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
#sleep 5

#Launch the edited view_navigation.launch with Add to observe the map in Rviz.
xterm -e " roslaunch add_markers home_service_view.launch rviz_path:=/home/workspace/catkin_ws/src/rvizConfig/home_service_Rviz.rviz" &
sleep 5

#Launch pick_objcts node.
xterm -e " rosrun pick_objects pick_objects " &
sleep 5

#Launch add_markers node.
xterm -e " rosrun add_markers add_markers " 
