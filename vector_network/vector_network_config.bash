#!/bin/bash

# VECTOR1 is ROS master
#   Interface: is eth0
#   IP address is 10.66.171.2

# A remote PC is ROS slave, the default interface is lo (change it to whatever physical interface is connected to vector if working with real hardware), 
#    and the IP address is DHCP from onboard wireless

if [ "$HOSTNAME" = VECTOR1 ]; then
    export ROBOT_NETWORK=eth0
    export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
    export ROS_MASTER_URI=http://$ROS_IP:11311/
else
    # This should be changed to whatever physical interface is connected to the robot (ie wlan0, eth0, etc..)
    # or left as-is for simulation
    #export ROBOT_NETWORK=wlan0
    #export ROBOT_NETWORK=eth0
    export ROBOT_NETWORK=lo
    export ROS_IP=$(ip -4 address show $ROBOT_NETWORK | grep 'inet' | sed 's/.*inet \([0-9\.]\+\).*/\1/')
    
    # This should be changed to the following if connected to a real robot and the line after commented out
    # you need to define MOVO2 in /etc/hosts (normally this would be adding the line 'VECTOR1 10.66.171.2')
    #export ROS_MASTER_URI=http://VECTOR1:11311/
    
    #By default we setup for simulation (comment out if working with real robot)
    export ROS_MASTER_URI=http://$ROS_IP:11311/
fi

