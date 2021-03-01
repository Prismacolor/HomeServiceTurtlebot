#!/bin/sh
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/my_map.yaml

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 3
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 3
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &

