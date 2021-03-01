#!/bin/sh
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/my_map.yaml

# xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=~/catkin_ws2/src/World/new_world " &
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 10
xterm -e " rosrun pick_objects pick_objects "
