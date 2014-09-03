if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros svn https://roboticssrv.wtb.tue.nl/svn/data/mirror/wg-ros-pkg/control_toolbox .
elif [[ $TUE_ROS_DISTRO == "hydro" ]]
then
    tue-install-ros system pr2-controllers
fi
