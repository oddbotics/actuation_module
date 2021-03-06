#!/bin/bash
source /opt/ros/indigo/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

master=$(ip addr show eth0 | grep -o '10.0.[[:digit:]].[[:digit:]]' | head -1 |
awk '{
        split($1, a, ".")
        subnet = a[3]
        print "10.0." subnet ".1:11311"
}')


export ROS_MASTER_URI=http://$master
export ROS_HOSTNAME=motor-2
export ROS_IP=$(ip addr show eth0 | grep -o '10.0.[[:digit:]].[[:digit:]]' | head -1)

sleep 2;

location=$(ip addr show eth0 | grep -o '10.0.[[:digit:]].[[:digit:]]' | head -1 |
awk '{
	split($1, a, ".")
	subnet = a[3]
	print "connector_" subnet
}')
conn_num=$(ip addr show eth0 | grep -o '10.0.[[:digit:]].[[:digit:]]' | head -1 |
awk '{
        split($1, a, ".")
        subnet = a[3]
        print subnet
}')


roslaunch actuation_module actuation.launch actuation_folder:=dc_motor_1 actuation_type:=dc_motor position:=$location control_type:=velocity connector_num:=$conn_num
