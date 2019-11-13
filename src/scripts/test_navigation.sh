#!/bin/sh
xterm  -e  " roslaunch my_robot turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot amcl_world.launch" &
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch "
