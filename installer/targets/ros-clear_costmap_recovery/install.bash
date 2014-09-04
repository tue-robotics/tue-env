if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros system navigation
else
    tue-install-ros git https://github.com/tue-robotics/navigation.git clear_costmap_recovery
fi
