#!/bin/sh
xterm  -e  " roslaunch my_robot turtlebot_world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot amcl_world.launch" &
sleep 5
xterm  -e  " roslaunch my_robot view_navigation_marker.launch "&
sleep 5
xterm  -e  " rosrun add_markers add_markers "
