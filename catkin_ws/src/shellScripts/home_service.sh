#!/bin/sh
xterm -e "roslaunch ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/amcl_demo.launch" &
sleep 5
xterm -e "roslaunch ~/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch" &
sleep 5
xterm -e "rosrun add_markers add_markers_node" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node"
