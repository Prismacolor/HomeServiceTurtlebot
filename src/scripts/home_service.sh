#!/bin/sh
# export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find World)/new_world
# export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/my_map.yaml
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch " &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch " &
sleep 5
# xterm -e " rosrun rviz rviz -d /home/workspace/catkin_ws2/src/rvizConfig/home_service2.rviz " &
# xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch " &
xterm -e " rosrun rviz rviz -d `rospack find rvizConfig`/rviz/home_service3.rviz " &
sleep 5
xterm -e " rosrun add_markers add_markers " &
sleep 5
xterm -e " rosrun pick_objects pick_objects " 

