if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros-rosbuild svn https://roboticssrv.wtb.tue.nl/svn/ros/code/tue-ros-pkg/trunk/tue_msgs/rtt_amigo_msgs
else
    tue-install-ros svn https://roboticssrv.wtb.tue.nl/svn/ros/trunk/rtt_amigo_msgs .
fi
