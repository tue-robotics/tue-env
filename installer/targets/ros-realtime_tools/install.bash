if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros svn https://roboticssrv.wtb.tue.nl/svn/data/mirror/wg-ros-pkg/realtime_tools .
elif [[ $TUE_ROS_DISTRO == "hydro" ]]
then
    tue-install-ros system ros-control
fi
