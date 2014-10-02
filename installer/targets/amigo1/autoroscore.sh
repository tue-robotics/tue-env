#!/bin/bash

# Check argument
if [ -z "$1" ]
  then
    echo "No pid supplied"
fi

# Create dummy graphic environment
#PS1=rommel

# Source ros stuff as user is accustomed to
#source /home/amigo/.bashrc
TUE_ROS_DISTRO=hydro
source /home/amigo/.tue/setup.bash

# Launch the file
roslaunch --pid $1 --core

