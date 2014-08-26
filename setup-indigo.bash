export TUE_ROS_DISTRO=indigo

source /opt/ros/$TUE_ROS_DISTRO/setup.bash

if [ -f ~/ros/$ROS_DISTRO/catkin_ws/devel/setup.bash ]
then
    source ~/ros/$ROS_DISTRO/catkin_ws/devel/setup.bash
else
    cd ~/ros/$ROS_DISTRO/catkin_ws
    catkin_make
    source ~/ros/$ROS_DISTRO/catkin_ws/devel/setup.bash
fi

source ~/.tue/setup/tue.bash_functions
