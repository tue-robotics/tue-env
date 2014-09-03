# GROOVY specific packages
if [[ $TUE_ROS_DISTRO == "groovy" ]]
then
    tue-install-target ros-amigo_skill_server
    tue-install-target ros-amigo_inverse_reachability
fi
