#!/bin/bash

# Check argument
if [ -z "$1" ]
  then
    echo "No pid supplied"
elif [ -z "$2" ]
  then
    echo "No package supplied"
elif [ -z "$3" ]
  then
    echo "No launchfile supplied"
fi

# Source ros stuff as user is accustomed to
TUE_ROS_DISTRO=indigo
export ROSLAUNCH_SSH_UNKNOWN=1
source ~/.tue/setup.bash

# Launch the file
roslaunch --pid $1 --wait $2 $3

