#!/bin/bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ifconfig CHANGE 192.168.1.50 netmask 255.255.255.0 #change
ros2 run controller_pkg controller
