if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros svn https://roboticssrv.wtb.tue.nl/svn/ros/code/tue-ros-pkg/trunk/tue_navigation/base_local_planner
else
    tue-install-ros git https://github.com/tue-robotics/navigation.git base_local_planner
fi
