#!/bin/sh

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
# world_file:=/home/workspace/catkin_ws2/src/world/new_world
sleep 3
xterm -e " rosrun gmapping slam_gmapping " &
sleep 3
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 3
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch " 
