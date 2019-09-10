#!/bin/sh


#Using test_slam.sh to deploy a turtlebot inside the environment, 
#control it with keyboard commands, interface it with a SLAM package,
#and visualize the map in Rviz.


#To test if it is able to manually perform SLAM by teleoperating the robot.

#Launch turtlebot_world.launch to deploy a turtlebot in the environment.
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5

#Launch the gmapping_demo.launch to perform SLAM.
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
#Or rosrun gmapping slam_gmapping
sleep 5

#Launch the view_navigation.launch to observe the map in Rviz.
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5

#Launch the keyboard_teleop.launch to manually control the robot with keyboard commands.
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch "
