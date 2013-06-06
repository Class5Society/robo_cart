#!/bin/bash

# Run this as root, from the directory containing it!
#
# USAGE: sudo ./install.bash
#

stackPath=./

export robot=robocart
export user=talos
export release=$(ls /opt/ros/ | tail -n1)

source helpers.bash

if [ -d /etc/ros ]; then
  # Remove existing setup file so we can use our own.
  rm -f /etc/ros/setup.bash
else
  # Create /etc/ros so that we can put our file in there.
  mkdir /etc/ros
fi

ln -s /home/${user}/ros_catkin_ws/devel/setup.bash /etc/ros/setup.bash
source /etc/ros/setup.bash
pushd `rospack find ${robot}_bringup`/upstart > /dev/null

install_job core eth1 11311

popd > /dev/null
