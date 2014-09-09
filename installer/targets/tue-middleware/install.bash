# GROOVY specific packages
if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-target ros-amigo_arm_navigation
    tue-install-target ros-tue_move_base_3d
fi
