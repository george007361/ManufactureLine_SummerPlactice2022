#!/bin/bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select controller_interface_pkg
source install/setup.bash
colcon build --packages-select controller_pkg
