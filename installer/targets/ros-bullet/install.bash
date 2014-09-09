if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-ros system bullet
else
    tue-install-system libbullet-dev
fi
