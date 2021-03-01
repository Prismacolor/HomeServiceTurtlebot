# HomeServiceTurtlebot

This is the final project in the Udacity Robotics Engineering Nanodegree. The goal of this project is to create a robot that maps its environment and then 
is able to navigate to a goal, pick up an object, and drop the object off at a specified location. 

The modules in this project are as follows:
World: This contains map and world files from a previous project which can be used instead of the Turtlebot environment.
add_markers: This allows you to add markers to the environment for the robot to pick up and drop off. 
map: Contains a map of the Turtlebot world.
pick_objects: Sets the code for the robot to pick up items from the goal locations.
rvizConfig: Contains an rviz configuration with the marker set in the Turtlebot world.
scripts: Contains sh scripts for running the different nodes of this project.
slam_gmapping: This is used for localization and to help the robot create a map of its world.
turtlebot: This includes a teleop package that allows you to move the Turtlebot.
turtlebot_interactions: Allows you to launch a rviz simulation of the Turtlebot world. 
turtlebot_simulator: This launches a Gazebo model of the Turtlebot world. 
urdf: Contains files for another robotI built in previous projects, which can be used here instead of the Turtlebot. 
wall_follower: creates a localization node that allows the robot to follow along walls. 

The main script for this project is located in the scripts folder: home_service.sh. This spawns a Turtlebot in its environment as well as a nearby marker, created 
by the add_markers node. The robot receives a pick up goal location from the pick_objects node. It navigates to the goal and picks up the marker. It then receives 
a drop off location. Finally the robot moves to the drop off location to drop off the object. 

