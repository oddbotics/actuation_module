#!/bin/bash
source /opt/ros/indigo/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://odroid:11311
export ROS_HOSTNAME=motor-2

sleep 2;

address=$(ip addr show eth0 | grep -o '10.0.[[:digit:]].[[:digit:]]' | head -1 |
awk '{
	split($1, a, ".")
	subnet = a[3]
	print "connector_" subnet
}')

roslaunch actuation_module actuation.launch actuation_folder:=dc_motor_1 actuation_type:=motor position:=$address control_type:=velocity
