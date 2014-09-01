if [ "$ROS_DISTRO_DEFAULT" == "groovy" ]
then
    source ~/ros/groovy/catkin_ws/src/tue/trunk/amigo_admin_files/bash/amigo_user.bashrc
else

    source ~/.tue/setup/tue.bash_functions
    if [ -f ~/.tue/setup/target_setup.bash ]
    then
        source ~/.tue/setup/target_setup.bash
    else
        tue-setup
    fi
fi
