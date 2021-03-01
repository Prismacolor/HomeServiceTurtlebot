#!/bin/sh
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/my_map.yaml
# map_file:=/home/workspace/catkin_ws2/src/map/my_map.yaml
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
sleep 5
xterm -e " rosrun add_markers add_markers "
