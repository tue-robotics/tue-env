export TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Load tue-env tool
source $TUE_DIR/setup/tue-env.bash

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
if [ "$TUE_ENV" == "hydro" ] || [ "$TUE_ENV" == "indigo" ]
then
    mkdir -p $TUE_DIR/user/envs
    [ -f $TUE_DIR/user/envs/$TUE_ENV ] || echo "$HOME/ros/$TUE_ENV" > $TUE_DIR/user/envs/$TUE_ENV
fi

if [ ! -f $TUE_DIR/user/config/default_env ]
then
    if [ -z "$TUE_ROS_DISTRO" ]
    then
        if [ -z "$TUE_ENV" ]
        then
            # No environment, so all environment specific setup below does not need to be sourced
            return
        else
            TUE_ROS_DISTRO=$TUE_ENV
        fi
    else
        TUE_ENV=$TUE_ROS_DISTRO
    fi

    export TUE_ROS_DISTRO=$TUE_ROS_DISTRO
    export TUE_ENV=$TUE_ENV

    mkdir -p $TUE_DIR/user/config
    echo "TUE_ENV" > $TUE_DIR/user/config/default_env
fi
# ------------------------------------------

if [ -z "$TUE_ENV" ]
then
    if [ ! -f $TUE_DIR/user/config/default_env ]
    then
        # No environment, so all environment specific setup below does not need to be sourced
        return
    fi

    export TUE_ENV=`cat $TUE_DIR/user/config/default_env`

    if [ ! -f $TUE_DIR/user/envs/$TUE_ENV ]
    then
        echo "[tue] No such environment: '$TUE_ENV'"
        return
    fi
fi

export TUE_ENV_DIR=`cat $TUE_DIR/user/envs/$TUE_ENV`

if [ ! -d $TUE_ENV_DIR ]
then
    echo "[tue] Environment directory '$TUE_ENV_DIR' (environment '$TUE_ENV') does not exist"
    return 1
fi

source $TUE_DIR/setup/tue.bash_functions

if [ -f $TUE_ENV_DIR/.env/setup/user_setup.bash ]
then
    source $TUE_ENV_DIR/.env/setup/user_setup.bash
fi

if [ -f $TUE_ENV_DIR/.env/setup/target_setup.bash ]
then
	source $TUE_ENV_DIR/.env/setup/target_setup.bash
fi

export TUE_BIN=~/.tue/bin
export PATH=$TUE_BIN:$PATH

# Make sure ROS can find cmake modules of non-ROS packages
export CMAKE_PREFIX_PATH=$TUE_ENV_DIR/cmake:$CMAKE_PREFIX_PATH
