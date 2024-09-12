#!/bin/bash

if [ -z "$1" ]; then
	echo "ERROR: Argument invalid for port number"
	exit 1
fi
# source /opt/ros/noetic/setup.bash
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM$1 _baud:=57600
