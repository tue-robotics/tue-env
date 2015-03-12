export TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

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

export TUE_BIN=~/.tue/bin
export PATH=$TUE_BIN:$PATH

# Make sure ROS can find cmake modules of non-ROS packages
export CMAKE_PREFIX_PATH=$TUE_ENV_DIR/cmake:$CMAKE_PREFIX_PATH

# ------------------------------------------
# Helper function for checking if all env vars are set
function _tue-check-env-vars
{
    [ -n "$TUE_DIR" ] && [ -n "$TUE_ENV" ] && [ -n "$TUE_ENV_DIR" ] \
	   && [ -n "$TUE_BIN" ] && [ -n "$TUE_ROS_DISTRO" ] && return 0   
	echo "[tue] Not all needed environment variables are set."
	return 1
}
export -f _tue-check-env-vars

# ------------------------------------------

# Temporarily: make sure the ~/ros/hydro and ~/ros/indigo environment can be found
mkdir -p $TUE_ENV/user/envs
[ ! -d ~/ros/hydro ] || [ -f $TUE_ENV/user/envs/hydro ] || echo "$HOME/ros/hydro" > $TUE_ENV/user/envs/hydro
[ ! -d ~/ros/indigo ] || [ -f $TUE_ENV/user/envs/indigo ] || echo "$HOME/ros/indigo" > $TUE_ENV/user/envs/indigo

