#!/bin/bash

ip="$(hostname -I|tr -d ' ')"

#Single Machine Configuration
#export ROS_HOSTNAME=localhost
#export ROS_MASTER_URI=http://localhost:11311

#Multi Machine Configuration
export ROS_HOSTNAME="$ip"
export ROS_MASTER_URI="http://$ip:11311"
export ROS_IP="$ip"

#Team Selection
source .team_source.sh
echo "TEAM : $TEAM"
#launchfile à lancer
#roslaunch test_teensy.launch 
#roslaunch test_hokuyo.launch
#roslaunch AbsLoc.launch
#roslaunch turtleros.launch
roslaunch root_launch.launch
