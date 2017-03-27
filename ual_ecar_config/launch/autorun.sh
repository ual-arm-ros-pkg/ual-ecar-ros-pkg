#!/bin/bash 

# Put this file in the user's anacron like:
# @reboot bash /home/ual/catkin_ws/src/ual-ecar-ros-pkg/ual_ecar_config/launch/autorun.sh


source /opt/ros/kinetic/setup.bash
source /home/ual/catkin_ws/devel/setup.bash
roslaunch ual_ecar_config steering_low_level.launch
