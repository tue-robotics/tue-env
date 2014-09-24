if [ "$ROS_DISTRO_DEFAULT" == "groovy" ]
then
    source ~/ros/groovy/catkin_ws/src/tue/trunk/amigo_admin_files/bash/amigo_user.bashrc
else

    if [ -z "$TUE_ROS_DISTRO" ]
    then
		if [ -z "$TUE_ENV" ]
		then
        	echo "[tue] Please set TUE_ROS_DISTRO or TUE_ENV"
	        return
		else
			TUE_ROS_DISTRO=$TUE_ENV
		fi
	else
		TUE_ENV=$TUE_ROS_DISTRO
	fi

    export TUE_ROS_DISTRO=$TUE_ROS_DISTRO
	export TUE_ENV=$TUE_ENV
	export TUE_ENV_DIR=~/ros/$TUE_ENV

    source ~/.tue/setup/tue.bash_functions

    if [ -f $TUE_ENV_DIR/.env/setup/target_setup.bash ]
	then
		source $TUE_ENV_DIR/.env/setup/target_setup.bash
	elif [ -f ~/.tue/env/setup/target_setup.bash ]
    then
        source ~/.tue/env/setup/target_setup.bash
    fi
fi
