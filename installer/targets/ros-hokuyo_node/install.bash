if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros system laser-drivers
else
    tue-install-ros system hokuyo-node
    tue-install-ros system laser-filters
fi
