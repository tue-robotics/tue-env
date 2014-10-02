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


# Create dummy graphic environment
#PS1=rommel

# Source ros stuff as user is accustomed to
#source /home/amigo/.bashrc
source /home/amigo/.tue/setup.bash

# Launch the file
roslaunch --pid $1 --wait $2 $3

