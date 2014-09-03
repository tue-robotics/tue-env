if [ "$ROS_DISTRO_DEFAULT" == "groovy" ]
then
    source ~/ros/groovy/catkin_ws/src/tue/trunk/amigo_admin_files/bash/amigo_user.bashrc
else

    if [ -z "$TUE_ROS_DISTRO" ]
    then
        echo "[tue] Please set TUE_ROS_DISTRO"
        return
    else
        export TUE_ROS_DISTRO=$TUE_ROS_DISTRO
    fi

    source ~/.tue/setup/tue.bash_functions
    if [ -f ~/.tue/setup/target_setup.bash ]
    then
        source ~/.tue/setup/target_setup.bash
    else
        tue-setup
    fi
fi
