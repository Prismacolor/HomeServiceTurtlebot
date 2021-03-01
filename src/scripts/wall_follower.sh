#!/bin/sh

# xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=~/catkin_ws2/src/World/new_world. "
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch " &
sleep 5
xterm -e " roslaunch turtlebot_rviz.launchers view_navigation.launch " &
sleep 10
xterm -e  " rosrun wall_follower wall_follower "
