source /opt/ros/hydro/setup.bash

if [ -f ~/ros/hydro/catkin_ws/devel/setup.bash ]
then
    source ~/ros/hydro/catkin_ws/devel/setup.bash
else
    cd ~/ros/hydro/catkin_ws
    catkin_make
    source ~/ros/hydro/catkin_ws/devel/setup.bash
fi

source ~/.tue/setup/tue.bash_functions
