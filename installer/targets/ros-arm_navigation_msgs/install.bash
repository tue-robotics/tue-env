if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros system arm-navigation
else
    tue-install-ros git TODO
fi
