#!/bin/sh
xterm -e "roslaunch ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch ~/catkin_ws/src/turtlebot_simulator/turtlebot_gazebo/launch/gmapping_demo.launch" &
sleep 5
xterm -e "roslaunch ~/catkin_ws/src/turtlebot_interactions/turtlebot_rviz_launchers/launch/view_navigation.launch" &
sleep 5
cd ~/catkin_ws/
xterm -e "rosrun wall_follower wall_follower_node"
