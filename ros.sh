#!/bin/bash

# IP / hostname configuration
ip="$(hostname -I|tr -d ' ')"
if [ -z "$ip" ]; then
	ip=127.0.0.1
fi

export ROS_HOSTNAME="$ip"
export ROS_MASTER_URI="http://$ip:11311"
export ROS_IP="$ip"

#Team Selection
source .team_source.sh
echo "TEAM : $TEAM"

# Re-source ros conf (just in case)
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

#launchfile à lancer
#roslaunch test_teensy.launch 
#roslaunch test_hokuyo.launch
#roslaunch AbsLoc.launch
#roslaunch turtleros.launch
roslaunch root_launch.launch
