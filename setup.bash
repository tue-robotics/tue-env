export TUE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Load tue-env tool
source $TUE_DIR/setup/tue-env.bash

# ------------------------------------------
# Helper function for checking if all env vars are set
function _tue-check-env-vars
{
    [ -n "$TUE_DIR" ] && [ -n "$TUE_ENV" ] && [ -n "$TUE_ENV_DIR" ] \
       && [ -n "$TUE_BIN" ] && return 0   
    echo "[tue] Not all needed environment variables are set."
    return 1
}
export -f _tue-check-env-vars

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

# -----------------------------------------
# Fix annoying perl language warnings
export LC_ALL="C.UTF-8"

# -----------------------------------------
# Load all the bash functions
source $TUE_DIR/setup/tue.bash_functions

if [ -f $TUE_ENV_DIR/.env/setup/user_setup.bash ]
then
    source $TUE_ENV_DIR/.env/setup/user_setup.bash
fi

if [ -f $TUE_ENV_DIR/.env/setup/target_setup.bash ]
then
    source $TUE_ENV_DIR/.env/setup/target_setup.bash
fi

if [ -f $TUE_DIR/setup/tue.bash_aliases ]
then
    source $TUE_DIR/setup/tue.bash_aliases
fi

export TUE_BIN=$TUE_DIR/bin
export PATH=$TUE_BIN:$PATH

export TUE_ENV=$TUE_ENV

# Make sure ROS can find cmake modules of non-ROS packages
#export CMAKE_PREFIX_PATH=$TUE_ENV_DIR/cmake:$CMAKE_PREFIX_PATH
export CURRENT_CMAKE_BUILD_DIR="$(catkin locate --workspace $TUE_SYSTEM_DIR --build)"
