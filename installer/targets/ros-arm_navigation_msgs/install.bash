if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros system arm-navigation
else
    tue-install-ros git https://github.com/PR2/arm_navigation_msgs.git
fi
